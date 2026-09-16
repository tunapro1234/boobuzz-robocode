package boobuzz.core.debug;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Non-blocking broadcast tap for line-oriented debug records.
 *
 * <p>The robot loop only enqueues a line.  Each client has an independent
 * bounded queue and writer thread, so a slow laptop cannot stall control.
 */
public final class DebugTap implements AutoCloseable {

    private static final int QUEUE_CAPACITY = 256;

    private final int port;
    private final ServerSocket server;
    private final Set<Client> clients = ConcurrentHashMap.newKeySet();
    private final AtomicLong dropped = new AtomicLong();
    private final Thread acceptThread;
    private volatile boolean running = true;

    /** Creates a disabled tap when {@code port == 0}; otherwise binds immediately. */
    public DebugTap(int port) throws IOException {
        if (port < 0 || port > 65535) {
            throw new IllegalArgumentException("debug tap port out of range: " + port);
        }
        this.port = port;
        if (port == 0) {
            server = null;
            acceptThread = null;
            running = false;
            return;
        }
        ServerSocket opened = new ServerSocket();
        opened.setReuseAddress(true);
        opened.bind(new java.net.InetSocketAddress(port));
        server = opened;
        acceptThread = new Thread(this::acceptLoop, "debug-tap-accept-" + port);
        acceptThread.setDaemon(true);
        acceptThread.start();
    }

    public int port() {
        return port;
    }

    public boolean enabled() {
        return server != null && running;
    }

    public int clientCount() {
        return clients.size();
    }

    public long droppedCount() {
        return dropped.get();
    }

    /** Enqueues one complete line for every connected listener. */
    public void publish(String line) {
        if (!enabled() || line == null) {
            return;
        }
        for (Client client : clients) {
            client.offer(line);
        }
    }

    @Override
    public void close() {
        if (!running) {
            return;
        }
        running = false;
        try {
            server.close();
        } catch (IOException ignored) {
            // Closing an already closed socket is harmless during shutdown.
        }
        for (Client client : clients.toArray(Client[]::new)) {
            remove(client);
        }
        if (acceptThread != null) {
            acceptThread.interrupt();
        }
    }

    private void acceptLoop() {
        while (running) {
            try {
                Socket socket = server.accept();
                socket.setTcpNoDelay(true);
                Client client = new Client(socket);
                clients.add(client);
                client.start();
            } catch (SocketException e) {
                if (running) {
                    // A transient accept failure must not enter a busy loop.
                    Thread.yield();
                }
            } catch (IOException e) {
                if (running) {
                    Thread.yield();
                }
            }
        }
    }

    private void remove(Client client) {
        if (clients.remove(client)) {
            client.close();
        }
    }

    private final class Client implements AutoCloseable {
        private final Socket socket;
        private final ArrayBlockingQueue<String> queue = new ArrayBlockingQueue<>(QUEUE_CAPACITY);
        private final Thread writer;
        private volatile boolean open = true;

        private Client(Socket socket) {
            this.socket = socket;
            writer = new Thread(this::writeLoop,
                    "debug-tap-client-" + socket.getRemoteSocketAddress());
            writer.setDaemon(true);
        }

        private void start() {
            writer.start();
        }

        private void offer(String line) {
            if (!open || queue.offer(line)) {
                return;
            }
            queue.poll();
            if (!queue.offer(line)) {
                dropped.incrementAndGet();
            } else {
                dropped.incrementAndGet();
            }
        }

        private void writeLoop() {
            try (BufferedWriter out = new BufferedWriter(new OutputStreamWriter(
                    socket.getOutputStream(), StandardCharsets.UTF_8))) {
                while (open) {
                    String line = queue.take();
                    out.write(line);
                    out.newLine();
                    out.flush();
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } catch (IOException ignored) {
                // The tap is best-effort; the next publish removes this client.
            } finally {
                open = false;
                clients.remove(this);
                try {
                    socket.close();
                } catch (IOException ignored) {
                    // Already closed.
                }
            }
        }

        @Override
        public void close() {
            open = false;
            writer.interrupt();
            try {
                socket.close();
            } catch (IOException ignored) {
                // Already closed.
            }
        }
    }
}
