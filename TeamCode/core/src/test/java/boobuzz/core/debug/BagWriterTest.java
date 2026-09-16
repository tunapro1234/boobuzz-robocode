package boobuzz.core.debug;

import org.junit.Test;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class BagWriterTest {

    @Test
    public void writesHeaderAndThreeLineGroups() throws Exception {
        Path path = Files.createTempFile("robot-bag", ".jsonl");
        try (BagWriter writer = new BagWriter(path, "cplx1", "auto", "abc")) {
            writer.writeLines(List.of("{\"seam\":\"hal\"}",
                    "{\"seam\":\"subsystem\"}", "{\"seam\":\"logic\"}"));
        }
        List<String> lines = Files.readAllLines(path);
        assertEquals(4, lines.size());
        assertEquals(1.0, JsonCodec.num(JsonCodec.parseObject(lines.get(0)), "bag", 0), 0.0);
        assertTrue(lines.get(3).contains("logic"));
        Files.deleteIfExists(path);
    }
}
