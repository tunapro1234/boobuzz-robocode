package boobuzz.sim;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * A test child process whose merged output is drained on a daemon thread, so it
 * can never block on a full pipe, and whose listening port is read from the line
 * it prints after binding. Children bind port 0 and report the OS-assigned port:
 * a probed "free" port can be taken by a concurrent run before the child binds it.
 */
final class ChildProcess implements AutoCloseable {

    /** {@code python -m sim.server} logs this to stderr once its listener is bound. */
    static final Pattern SIM_SERVER_LISTENING =
            Pattern.compile("^sim: \\S+:(\\d+) listening\\b");
    /** SimMain prints this once its SocketController is bound. */
    static final Pattern SIM_MAIN_CONTROL_PORT =
            Pattern.compile("^controller: socket \\(port (\\d+),");

    private final Process process;
    private final StringBuffer output = new StringBuffer();
    private final CompletableFuture<Integer> port = new CompletableFuture<>();
    private final CompletableFuture<Void> drained = new CompletableFuture<>();

    private ChildProcess(Process process, Pattern portLine, String name) {
        this.process = process;
        Thread reader = new Thread(() -> drain(portLine), "test-child-output-" + name);
        reader.setDaemon(true);
        reader.start();
    }

    static ChildProcess start(List<String> command, File directory, Pattern portLine,
                              String name) throws IOException {
        Process process = new ProcessBuilder(command)
                .directory(directory)
                .redirectErrorStream(true)
                .start();
        return new ChildProcess(process, portLine, name);
    }

    /** Starts sim.server on port 0 and returns once it is listening. */
    static ChildProcess startSimServer(File simulator, File python, File mechanism)
            throws Exception {
        ChildProcess server = start(List.of(
                python.getAbsolutePath(), "-m", "sim.server",
                "--mechanism", mechanism.getAbsolutePath(),
                "--physics", "pymunk", "--headless", "--port", "0"),
                simulator, SIM_SERVER_LISTENING, "sim-server");
        server.awaitPort(10_000L);
        return server;
    }

    Process process() {
        return process;
    }

    /**
     * Blocks until the child printed its port line. Fails if the child exits (or
     * closes its output) first; the timeout only bounds a hung child.
     */
    int awaitPort(long timeoutMs) throws Exception {
        try {
            return port.get(timeoutMs, TimeUnit.MILLISECONDS);
        } catch (ExecutionException e) {
            throw new IOException(e.getCause().getMessage() + "; output:\n" + output);
        } catch (TimeoutException e) {
            throw new IOException("child printed no port line within " + timeoutMs
                    + " ms; output:\n" + output);
        }
    }

    /** Waits for exit and for the drain thread to reach end of output. */
    String awaitExitOutput(long timeoutMs) throws Exception {
        if (!process.waitFor(timeoutMs, TimeUnit.MILLISECONDS)) {
            throw new IOException("child did not exit within " + timeoutMs
                    + " ms; output:\n" + output);
        }
        drained.get(timeoutMs, TimeUnit.MILLISECONDS);
        return output.toString();
    }

    String output() {
        return output.toString();
    }

    private void drain(Pattern portLine) {
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(
                process.getInputStream(), StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) {
                output.append(line).append('\n');
                if (portLine != null && !port.isDone()) {
                    Matcher matcher = portLine.matcher(line);
                    if (matcher.find()) port.complete(Integer.parseInt(matcher.group(1)));
                }
            }
        } catch (IOException ignored) {
            // Process teardown closes the stream.
        } finally {
            port.completeExceptionally(new IOException("child output ended before its port line"));
            drained.complete(null);
        }
    }

    @Override
    public void close() throws InterruptedException {
        if (!process.isAlive()) return;
        process.destroy();
        if (!process.waitFor(2, TimeUnit.SECONDS)) {
            process.destroyForcibly();
            process.waitFor(2, TimeUnit.SECONDS);
        }
    }
}
