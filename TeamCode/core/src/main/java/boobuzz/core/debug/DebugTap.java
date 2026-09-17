package boobuzz.core.debug;

import com.pedropathing.math.Pose;

import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;
import java.util.LinkedHashMap;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

/**
 * One bounded, asynchronous sink for debug tap clients and JSONL bags.
 *
 * <p>The robot loop only calls {@link #offer(DebugFrame)}. The dispatcher is
 * the sole thread that serializes records and writes the bag; per-client workers
 * only transport already-serialized lines to their sockets.
 */
public final class DebugTap implements AutoCloseable {

    private static final int QUEUE_CAPACITY = 512;
    private static final int CLIENT_QUEUE_CAPACITY = 512;
    private static final int DROP_REPORT_INTERVAL = 64;
    private static final long CLIENT_WRITE_DEADLINE_NANOS =
            TimeUnit.MILLISECONDS.toNanos(1000);

    private final int port;
    private final ServerSocket server;
    private final Set<Client> clients = ConcurrentHashMap.newKeySet();
    private final ArrayBlockingQueue<DebugFrame> queue = new ArrayBlockingQueue<>(QUEUE_CAPACITY);
    private final AtomicLong dropped = new AtomicLong();
    private final Thread acceptThread;
    private final Thread dispatchThread;
    private volatile BagSpec bagSpec;
    private volatile boolean running = true;
    private volatile String bagError;
    private BagWriter bagWriter;
    private BagSpec activeBag;
    private long dispatchedFrames;
    private long reportedDrops;

    /** Creates a dispatcher; port zero leaves the network tap disabled. */
    public DebugTap(int port) throws IOException {
        if (port < 0 || port > 65535) {
            throw new IllegalArgumentException("debug tap port out of range: " + port);
        }
        this.port = port;
        if (port == 0) {
            server = null;
            acceptThread = null;
        } else {
            ServerSocket opened = new ServerSocket();
            opened.setReuseAddress(true);
            opened.bind(new java.net.InetSocketAddress(port));
            server = opened;
            acceptThread = new Thread(this::acceptLoop, "debug-tap-accept-" + port);
            acceptThread.setDaemon(true);
            acceptThread.start();
        }
        dispatchThread = new Thread(this::dispatchLoop, "debug-tap-dispatch");
        dispatchThread.setDaemon(true);
        dispatchThread.start();
    }

    public int port() {
        return port;
    }

    /** True when a network listener is active; a port-zero instance may still bag. */
    public boolean enabled() {
        return server != null && running;
    }

    public int clientCount() {
        return clients.size();
    }

    public long droppedCount() {
        return dropped.get();
    }

    public String bagError() {
        return bagError;
    }

    /** Configures an asynchronous bag; no file is opened by the caller. */
    public void configureBag(File path, String engine, String controller,
                             String constantsHash, Pose startPose) {
        bagSpec = path == null ? null
                : new BagSpec(path, engine, controller, constantsHash, startPose);
    }

    /** Enqueues immutable references without serializing or doing I/O. */
    public void offer(DebugFrame frame) {
        if (frame == null || !running) {
            return;
        }
        if (queue.offer(frame)) {
            return;
        }
        // Keep the newest control tick and account for the discarded oldest one.
        queue.poll();
        dropped.incrementAndGet();
        if (!queue.offer(frame)) {
            dropped.incrementAndGet();
        }
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
        if (acceptThread != null) {
            acceptThread.interrupt();
            try {
                acceptThread.join(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        dispatchThread.interrupt();
        try {
            dispatchThread.join(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        if (dispatchThread.isAlive() && bagError == null) {
            bagError = "debug tap dispatcher did not terminate during close";
        }
        for (Client client : clients.toArray(Client[]::new)) {
            remove(client);
        }
    }

    private void acceptLoop() {
        while (running) {
            try {
                Socket socket = server.accept();
                socket.setTcpNoDelay(true);
                clients.add(new Client(socket));
            } catch (SocketException e) {
                if (running) Thread.yield();
            } catch (IOException e) {
                if (running) Thread.yield();
            }
        }
    }

    private void dispatchLoop() {
        try {
            while (running || !queue.isEmpty()) {
                ensureBag();
                DebugFrame frame = queue.poll(100, TimeUnit.MILLISECONDS);
                if (frame == null) {
                    continue;
                }
                // Configuration may race the first offered tick; resolve it before
                // serializing so the first seam is never silently omitted from a bag.
                ensureBag();
                List<String> lines = Arrays.asList(
                        SeamJson.hal(frame.state().t(), frame.state(), frame.action()),
                        SeamJson.subsystem(frame.state().t(), frame.calls(), frame.action().events()),
                        SeamJson.logic(frame.state().t(), frame.feedback(), frame.batch()));
                writeLines(lines);
                dispatchedFrames++;
                long dropCount = dropped.get();
                if (dropCount != reportedDrops
                        && (dispatchedFrames % DROP_REPORT_INTERVAL == 0 || !running)) {
                    enqueueClients(dropLine(dropCount));
                    reportedDrops = dropCount;
                }
            }
            ensureBag();
            long dropCount = dropped.get();
            if (dropCount != reportedDrops) {
                enqueueClients(dropLine(dropCount));
                reportedDrops = dropCount;
            }
        } catch (InterruptedException e) {
            // close() interrupts polling; drain the already-enqueued frames first.
            while (!queue.isEmpty()) {
                DebugFrame frame = queue.poll();
                if (frame == null) break;
                try {
                    ensureBag();
                    writeLines(Arrays.asList(
                            SeamJson.hal(frame.state().t(), frame.state(), frame.action()),
                            SeamJson.subsystem(frame.state().t(), frame.calls(), frame.action().events()),
                            SeamJson.logic(frame.state().t(), frame.feedback(), frame.batch())));
                } catch (IOException ignored) {
                    break;
                }
            }
            try {
                ensureBag();
                long dropCount = dropped.get();
                if (dropCount != reportedDrops) enqueueClients(dropLine(dropCount));
            } catch (IOException ignored) {
                // Shutdown is best effort.
            }
            Thread.currentThread().interrupt();
        } catch (IOException e) {
            bagError = e.getMessage() == null ? e.toString() : e.getMessage();
        } finally {
            if (bagWriter != null) {
                try {
                    closeBag();
                } catch (IOException e) {
                    if (bagError == null) {
                        bagError = e.getMessage() == null ? e.toString() : e.getMessage();
                    }
                }
            }
            for (Client client : clients.toArray(Client[]::new)) {
                remove(client);
            }
        }
    }

    private void ensureBag() throws IOException {
        BagSpec requested = bagSpec;
        if (requested == activeBag) {
            return;
        }
        if (bagWriter != null) {
            closeBag();
        }
        activeBag = requested;
        if (requested != null) {
            try {
                bagWriter = new BagWriter(requested.path(), requested.engine(),
                        requested.controller(), requested.constantsHash(), requested.startPose());
            } catch (IOException e) {
                bagError = e.getMessage() == null ? e.toString() : e.getMessage();
                throw e;
            }
        }
    }

    private void closeBag() throws IOException {
        if (bagWriter == null) {
            return;
        }
        String error = bagWriter.closeWithTapDrops(dropped.get());
        bagWriter = null;
        if (error != null) {
            if (bagError == null) {
                bagError = error;
            }
            throw new IOException(error);
        }
    }

    private void writeLines(List<String> lines) throws IOException {
        // The bag is independent of tap-client backpressure and is written first.
        if (bagWriter != null) {
            bagWriter.writeLines(lines);
        }
        for (String line : lines) {
            enqueueClients(line);
        }
    }

    /** Enqueues tap lines without doing socket I/O on the dispatcher. */
    private void enqueueClients(String line) {
        for (Client client : clients.toArray(Client[]::new)) {
            client.offer(line);
        }
    }

    private String dropLine(long count) {
        LinkedHashMap<String, Object> line = new LinkedHashMap<>();
        line.put("tap_dropped", count);
        return JsonCodec.stringify(line);
    }

    private void remove(Client client) {
        if (clients.remove(client)) {
            client.close();
        }
    }

    private final class Client implements AutoCloseable {
        private final Socket socket;
        private final BufferedWriter writer;
        private final ArrayBlockingQueue<String> lines =
                new ArrayBlockingQueue<>(CLIENT_QUEUE_CAPACITY);
        private final Thread writerThread;
        private final Thread writeWatchdogThread;
        private volatile long writeStartedNanos;
        private volatile boolean open = true;

        private Client(Socket socket) throws IOException {
            this.socket = socket;
            writer = new BufferedWriter(new OutputStreamWriter(
                    socket.getOutputStream(), StandardCharsets.UTF_8));
            writerThread = new Thread(this::writeLoop,
                    "debug-tap-client-" + socket.getRemoteSocketAddress());
            writeWatchdogThread = new Thread(this::writeDeadlineLoop,
                    "debug-tap-client-watchdog-" + socket.getRemoteSocketAddress());
            writerThread.setDaemon(true);
            writeWatchdogThread.setDaemon(true);
            writerThread.start();
            writeWatchdogThread.start();
        }

        /** Adds one line, dropping this client's oldest line if it is full. */
        private void offer(String line) {
            if (!open || !running) {
                return;
            }
            if (lines.offer(line)) {
                return;
            }
            lines.poll();
            dropped.incrementAndGet();
            if (!lines.offer(line)) {
                dropped.incrementAndGet();
            }
        }

        private void writeLoop() {
            try {
                while (open || !lines.isEmpty()) {
                    String line = lines.poll(100, TimeUnit.MILLISECONDS);
                    if (line == null) {
                        continue;
                    }
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
                    remove(this);
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
                            && System.nanoTime() - started > CLIENT_WRITE_DEADLINE_NANOS) {
                        remove(this);
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
            if (!open) {
                return;
            }
            open = false;
            try {
                socket.shutdownOutput();
            } catch (IOException ignored) {
                // The socket may already be closed by a failed client.
            }
            try {
                socket.close();
            } catch (IOException ignored) {
                // Already closed.
            }
            writerThread.interrupt();
            if (writerThread != Thread.currentThread()) {
                try {
                    writerThread.join(500);
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
            try {
                writer.close();
            } catch (IOException ignored) {
                // Already closed.
            }
        }
    }

    private record BagSpec(File path, String engine, String controller,
                           String constantsHash, Pose startPose) {}
}
