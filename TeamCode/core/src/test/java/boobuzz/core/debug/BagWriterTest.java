package boobuzz.core.debug;

import org.junit.Test;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class BagWriterTest {

    @Test
    public void writesHeaderAndThreeLineGroups() throws Exception {
        Path path = Files.createTempFile("robot-bag", ".jsonl");
        try (BagWriter writer = new BagWriter(path.toFile(), "cplx1", "auto", "abc")) {
            writer.writeLines(List.of("{\"seam\":\"hal\"}",
                    "{\"seam\":\"subsystem\"}", "{\"seam\":\"logic\"}"));
        }
        List<String> lines = Files.readAllLines(path);
        assertEquals(4, lines.size());
        assertEquals(1.0, JsonCodec.num(JsonCodec.parseObject(lines.get(0)), "bag", 0), 0.0);
        assertTrue(lines.get(3).contains("logic"));
        Files.deleteIfExists(path);
    }

    @Test
    public void writesDropCountAsSeamShapedFooter() throws Exception {
        Path path = Files.createTempFile("robot-bag-drops", ".jsonl");
        try (BagWriter writer = new BagWriter(path.toFile(), "cplx1", "auto", "abc")) {
            writer.writeLines(List.of(
                    "{\"seam\":\"hal\",\"t_ms\":20}",
                    "{\"seam\":\"subsystem\",\"t_ms\":20}",
                    "{\"seam\":\"logic\",\"t_ms\":20}"));
            assertEquals(null, writer.closeWithTapDrops(7));
        }
        List<String> lines = Files.readAllLines(path);
        assertEquals(5, lines.size());
        for (int i = 1; i < lines.size(); i++) {
            Map<String, Object> record = JsonCodec.parseObject(lines.get(i));
            assertTrue(record.containsKey("seam"));
            assertTrue(record.containsKey("t_ms"));
        }
        Map<String, Object> footer = JsonCodec.parseObject(lines.get(4));
        assertEquals("meta", JsonCodec.str(footer, "seam", ""));
        assertEquals(7.0, JsonCodec.num(footer, "tap_dropped", 0), 0.0);
        Files.deleteIfExists(path);
    }
}
