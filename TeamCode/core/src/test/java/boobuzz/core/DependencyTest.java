package boobuzz.core;

import org.junit.Test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import static org.junit.Assert.assertTrue;

/** Keeps the core layer dependency direction explicit and mechanically checked. */
public class DependencyTest {

    private static final String CORE = "boobuzz.core.";
    @Test
    public void mainImportsFollowLayerRules() throws IOException {
        Path sourceRoot = Path.of("src/main/java");
        if (!Files.isDirectory(sourceRoot)) {
            sourceRoot = Path.of("TeamCode/core/src/main/java");
        }
        List<String> violations = new ArrayList<>();
        try (Stream<Path> files = Files.walk(sourceRoot)) {
            files.filter(path -> path.toString().endsWith(".java"))
                    .forEach(path -> scan(path, violations));
        }
        assertTrue("forbidden core imports: " + violations, violations.isEmpty());
    }

    private static void scan(Path path, List<String> violations) {
        try {
            List<String> lines = Files.readAllLines(path);
            String packageName = lines.stream()
                    .filter(line -> line.startsWith("package "))
                    .map(line -> line.substring("package ".length(), line.length() - 1))
                    .findFirst()
                    .orElse("");
            String layer = layer(packageName);
            for (String line : lines) {
                if (!line.startsWith("import " + CORE)) {
                    continue;
                }
                String imported = line.substring("import ".length(), line.length() - 1);
                if (!allowed(layer, imported)) {
                    violations.add(path + ": " + packageName + " -> " + imported);
                }
            }
        } catch (IOException e) {
            throw new AssertionError("could not read " + path, e);
        }
    }

    private static String layer(String packageName) {
        if (packageName.equals("boobuzz.core")) {
            return "root";
        }
        if (packageName.startsWith(CORE)) {
            String relative = packageName.substring(CORE.length());
            int dot = relative.indexOf('.');
            return dot < 0 ? "root" : relative.substring(0, dot);
        }
        return "external";
    }

    private static boolean allowed(String layer, String imported) {
        if (layer.equals("root")) {
            return true;
        }
        String importedLayer = imported.substring(CORE.length());
        int dot = importedLayer.indexOf('.');
        importedLayer = dot < 0 ? importedLayer : importedLayer.substring(0, dot);
        if (importedLayer.equals(layer)) {
            return true;
        }
        return switch (layer) {
            case "contract" -> false;
            case "hal" -> importedLayer.equals("contract");
            case "subsystem" -> importedLayer.equals("contract")
                    || (importedLayer.equals("hal") && isAllowedSubsystemHal(imported));
            case "logic" -> importedLayer.equals("contract") || importedLayer.equals("subsystem")
                    || imported.equals("boobuzz.core.hal.RobotConstants");
            case "controller" -> importedLayer.equals("contract")
                    || imported.equals("boobuzz.core.hal.RobotConstants")
                    || importedLayer.equals("debug");
            default -> false;
        };
    }

    private static boolean isAllowedSubsystemHal(String imported) {
        return imported.equals("boobuzz.core.hal.Mechanism")
                || imported.equals("boobuzz.core.hal.RobotConstants");
    }
}
