package boobuzz.core.controller.socket;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.debug.JsonCodec;
import boobuzz.core.debug.SeamJson;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;
import java.util.LinkedHashMap;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Line-oriented external controller. Network work and JSON conversion stay on
 * background threads; {@link #decide(Feedback)} only reads atomics and queues a
 * feedback reference for the writer.
 *
 * <p>Each received line is a {@link RequestBatch}. Its stream is a level: every
 * tick reuses the newest received stream until the next line (or the watchdog)
 * replaces it. Its requests and cancels are edges: they reach {@code decide}
 * exactly once, so a client that sends one GOTO or SHOOT line gets one GOTO or
 * SHOOT, not one per tick until its next line. Lines that arrive between two
 * ticks are merged in arrival order.
 */
public final class SocketController implements boobuzz.core.controller.IController, AutoCloseable {

    private static final int FEEDBACK_QUEUE_CAPACITY = 64;
    private static final long CONNECTION_WRITE_DEADLINE_NANOS =
            TimeUnit.MILLISECONDS.toNanos(1000);

    private final int port;
    private final long timeoutNanos;
    private final ServerSocket server;
    private final Thread acceptThread;
    private final Thread feedbackDispatchThread;
    private final AtomicReference<Connection> connection = new AtomicReference<>();
    /** Newest received stream with no requests or cancels; returned every tick. */
    private final AtomicReference<RequestBatch> latest =
            new AtomicReference<>(RequestBatch.idle());
    /** Received requests and cancels that no tick has consumed yet, or null. */
    private final AtomicReference<RequestBatch> pending = new AtomicReference<>();
    private final AtomicLong batchesReceived = new AtomicLong();
    private final AtomicLong lastReceivedNanos = new AtomicLong();
    private final AtomicBoolean timeoutStopSent = new AtomicBoolean();
    private final ArrayBlockingQueue<Feedback> feedbackQueue =
            new ArrayBlockingQueue<>(FEEDBACK_QUEUE_CAPACITY);
    private final AtomicLong feedbackDrops = new AtomicLong();
    private volatile boolean running = true;
    private volatile String inputError;

    /** Uses the phase defaults from {@code RobotConstants}. */
    public SocketController() throws IOException {
        this(boobuzz.core.hal.RobotConstants.CONTROL_SOCKET_PORT,
                boobuzz.core.hal.RobotConstants.CONTROL_SOCKET_TIMEOUT_MS);
    }

    public SocketController(int port) throws IOException {
        this(port, boobuzz.core.hal.RobotConstants.CONTROL_SOCKET_TIMEOUT_MS);
    }

    public SocketController(int port, int timeoutMs) throws IOException {
        if (port < 0 || port > 65535) {
            throw new IllegalArgumentException("control socket port out of range: " + port);
        }
        if (timeoutMs < 0) {
            throw new IllegalArgumentException("control socket timeout must be non-negative");
        }
        this.port = port;
        timeoutNanos = TimeUnit.MILLISECONDS.toNanos(timeoutMs);
        if (port == 0) {
            server = null;
            acceptThread = null;
        } else {
            ServerSocket opened = new ServerSocket();
            opened.setReuseAddress(true);
            opened.bind(new java.net.InetSocketAddress(port));
            server = opened;
            acceptThread = new Thread(this::acceptLoop, "control-socket-accept-" + port);
            acceptThread.setDaemon(true);
            acceptThread.start();
        }
        feedbackDispatchThread = new Thread(this::writeLoop, "control-socket-feedback");
        feedbackDispatchThread.setDaemon(true);
        feedbackDispatchThread.start();
    }

    public int port() {
        return port;
    }

    public boolean enabled() {
        return server != null && running;
    }

    public boolean clientConnected() {
        Connection value = connection.get();
        return value != null && value.open;
    }

    public long feedbackDrops() {
        return feedbackDrops.get();
    }

    public String inputError() {
        return inputError;
    }

    /** Number of well-formed batch lines received since construction. */
    public long batchesReceived() {
        return batchesReceived.get();
    }

    @Override
    public RequestBatch decide(Feedback feedback) {
        if (feedback != null) {
            enqueueFeedback(feedback);
        }
        // Take the edges before reading the receive time: the reader publishes the
        // time first, so edges taken here are never older than the time checked.
        RequestBatch edges = pending.getAndSet(null);
        long received = lastReceivedNanos.get();
        if (received == 0L || System.nanoTime() - received > timeoutNanos) {
            // Stale edges are dropped with the stream: the watchdog cancels everything.
            if (received != 0L && timeoutStopSent.compareAndSet(false, true)) {
                return RequestBatch.cancelAll();
            }
            return RequestBatch.idle();
        }
        return edges != null ? edges : latest.get();
    }

    /** Merges two unconsumed batches: newer stream, requests and cancels in order. */
    private static RequestBatch merge(RequestBatch older, RequestBatch newer) {
        if (older == null) return newer;
        java.util.List<boobuzz.core.contract.Request> requests =
                new java.util.ArrayList<>(older.requests());
        requests.addAll(newer.requests());
        int[] a = older.cancels();
        int[] b = newer.cancels();
        int[] cancels = java.util.Arrays.copyOf(a, a.length + b.length);
        System.arraycopy(b, 0, cancels, a.length, b.length);
        return new RequestBatch(newer.stream(), requests, cancels);
    }

    @Override
    public void close() {
        if (!running) {
            return;
        }
        running = false;
        if (server != null) {
            try {
                server.close();
            } catch (IOException ignored) {
                // Shutdown is best effort.
            }
        }
        Connection current = connection.getAndSet(null);
        if (current != null) {
            current.close();
        }
        if (acceptThread != null) {
            acceptThread.interrupt();
            try {
                acceptThread.join(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        feedbackDispatchThread.interrupt();
        try {
            feedbackDispatchThread.join(1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void enqueueFeedback(Feedback feedback) {
        if (feedbackQueue.offer(feedback)) return;
        feedbackQueue.poll();
        feedbackDrops.incrementAndGet();
        if (!feedbackQueue.offer(feedback)) feedbackDrops.incrementAndGet();
    }

    private void acceptLoop() {
        while (running) {
            try {
                Socket socket = server.accept();
                socket.setTcpNoDelay(true);
                Connection next = new Connection(socket);
                Connection old = connection.getAndSet(next);
                if (old != null) old.close();
                next.readerThread.start();
            } catch (SocketException e) {
                if (running) Thread.yield();
            } catch (IOException e) {
                if (running) Thread.yield();
            }
        }
    }

    private void readLoop(Connection source) {
        try {
            String line;
            while (running && source.open && (line = source.reader.readLine()) != null) {
                if (line.trim().isEmpty()) continue;
                try {
                    RequestBatch batch = SeamJson.batchFrom(JsonCodec.parseObject(line));
                    latest.set(new RequestBatch(batch.stream(), null, null));
                    lastReceivedNanos.set(System.nanoTime());
                    timeoutStopSent.set(false);
                    pending.accumulateAndGet(batch, SocketController::merge);
                    batchesReceived.incrementAndGet();
                } catch (RuntimeException e) {
                    inputError = e.getMessage() == null ? e.toString() : e.getMessage();
                }
            }
        } catch (IOException e) {
            if (running) inputError = e.getMessage() == null ? e.toString() : e.getMessage();
        } finally {
            source.open = false;
            connection.compareAndSet(source, null);
            source.close();
        }
    }

    private void writeLoop() {
        try {
            while (running || !feedbackQueue.isEmpty()) {
                Feedback feedback = feedbackQueue.poll(100, TimeUnit.MILLISECONDS);
                if (feedback == null) continue;
                Connection target = connection.get();
                if (target == null || !target.open) continue;
                LinkedHashMap<String, Object> root = new LinkedHashMap<>();
                root.put("type", "feedback");
                root.put("t_ms", feedback.t());
                root.put("feedback", SeamJson.feedbackMap(feedback));
                target.offer(JsonCodec.stringify(root));
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            feedbackQueue.clear();
        }
    }

    private final class Connection implements AutoCloseable {
        private final Socket socket;
        private final BufferedReader reader;
        private final BufferedWriter writer;
        private final Thread readerThread;
        private final ArrayBlockingQueue<String> outbound =
                new ArrayBlockingQueue<>(FEEDBACK_QUEUE_CAPACITY);
        private final Thread writerThread;
        private final Thread writeWatchdogThread;
        private volatile long writeStartedNanos;
        private volatile boolean open = true;

        private Connection(Socket socket) throws IOException {
            this.socket = socket;
            reader = new BufferedReader(new InputStreamReader(
                    socket.getInputStream(), StandardCharsets.UTF_8));
            writer = new BufferedWriter(new OutputStreamWriter(
                    socket.getOutputStream(), StandardCharsets.UTF_8));
            readerThread = new Thread(() -> readLoop(this),
                    "control-socket-reader-" + socket.getRemoteSocketAddress());
            readerThread.setDaemon(true);
            writerThread = new Thread(this::writeLoop,
                    "control-socket-writer-" + socket.getRemoteSocketAddress());
            writeWatchdogThread = new Thread(this::writeDeadlineLoop,
                    "control-socket-writer-watchdog-" + socket.getRemoteSocketAddress());
            writerThread.setDaemon(true);
            writeWatchdogThread.setDaemon(true);
            writerThread.start();
            writeWatchdogThread.start();
        }

        private void offer(String line) {
            if (!open || !running) return;
            if (outbound.offer(line)) return;
            outbound.poll();
            feedbackDrops.incrementAndGet();
            if (!outbound.offer(line)) feedbackDrops.incrementAndGet();
        }

        private void writeLoop() {
            try {
                while (open || !outbound.isEmpty()) {
                    String line = outbound.poll(100, TimeUnit.MILLISECONDS);
                    if (line == null) continue;
                    writeStartedNanos = System.nanoTime();
                    writer.write(line);
                    writer.newLine();
                    writer.flush();
                    writeStartedNanos = 0L;
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } catch (IOException e) {
                writeStartedNanos = 0L;
                if (open) {
                    connection.compareAndSet(this, null);
                    close();
                }
            } finally {
                writeStartedNanos = 0L;
            }
        }

        private void writeDeadlineLoop() {
            try {
                while (open) {
                    long started = writeStartedNanos;
                    if (started != 0L
                            && System.nanoTime() - started > CONNECTION_WRITE_DEADLINE_NANOS) {
                        connection.compareAndSet(this, null);
                        close();
                        return;
                    }
                    Thread.sleep(25L);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        @Override
        public void close() {
            open = false;
            try {
                reader.close();
            } catch (IOException ignored) {
                // Already closed.
            }
            try {
                writer.close();
            } catch (IOException ignored) {
                // Already closed.
            }
            try {
                socket.close();
            } catch (IOException ignored) {
                // Already closed.
            }
            if (readerThread != Thread.currentThread()) {
                try {
                    readerThread.join(1000);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            writerThread.interrupt();
            if (writerThread != Thread.currentThread()) {
                try {
                    writerThread.join(1000);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            writeWatchdogThread.interrupt();
            if (writeWatchdogThread != Thread.currentThread()) {
                try {
                    writeWatchdogThread.join(500);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }
}
