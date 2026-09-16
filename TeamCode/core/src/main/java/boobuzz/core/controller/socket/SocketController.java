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
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Line-oriented external controller. Network work and JSON conversion stay on
 * background threads; {@link #decide(Feedback)} only reads atomics and queues a
 * feedback reference for the writer.
 */
public final class SocketController implements boobuzz.core.controller.IController, AutoCloseable {

    private static final int FEEDBACK_QUEUE_CAPACITY = 64;

    private final int port;
    private final long timeoutNanos;
    private final ServerSocket server;
    private final Thread acceptThread;
    private final Thread writerThread;
    private final AtomicReference<Connection> connection = new AtomicReference<>();
    private final AtomicReference<RequestBatch> latest =
            new AtomicReference<>(RequestBatch.idle());
    private final AtomicLong lastReceivedNanos = new AtomicLong();
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
        writerThread = new Thread(this::writeLoop, "control-socket-feedback");
        writerThread.setDaemon(true);
        writerThread.start();
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

    @Override
    public RequestBatch decide(Feedback feedback) {
        if (feedback != null) {
            enqueueFeedback(feedback);
        }
        long received = lastReceivedNanos.get();
        if (received == 0L || System.nanoTime() - received > timeoutNanos) {
            return RequestBatch.idle();
        }
        return latest.get();
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
        if (acceptThread != null) acceptThread.interrupt();
        writerThread.interrupt();
        try {
            writerThread.join(1000);
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
                if (line.isBlank()) continue;
                try {
                    latest.set(SeamJson.batchFrom(JsonCodec.parseObject(line)));
                    lastReceivedNanos.set(System.nanoTime());
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
                try {
                    target.write(JsonCodec.stringify(root));
                } catch (IOException e) {
                    connection.compareAndSet(target, null);
                    target.close();
                }
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
        }

        private synchronized void write(String line) throws IOException {
            if (!open) throw new SocketException("control socket is closed");
            writer.write(line);
            writer.newLine();
            writer.flush();
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
        }
    }
}
