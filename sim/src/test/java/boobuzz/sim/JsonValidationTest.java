package boobuzz.sim;

import org.junit.Test;

import static org.junit.Assert.assertThrows;

/** Protocol JSON helpers fail closed instead of silently substituting malformed values. */
public class JsonValidationTest {

    @Test
    public void rejectsWrongScalarTypes() {
        assertThrows(SimProtocolException.class,
                () -> Json.num(Json.parseObject("{\"x\":\"1\"}"), "x", 0));
        assertThrows(SimProtocolException.class,
                () -> Json.bool(Json.parseObject("{\"x\":1}"), "x"));
        assertThrows(SimProtocolException.class,
                () -> Json.str(Json.parseObject("{\"x\":1}"), "x"));
        assertThrows(SimProtocolException.class,
                () -> Json.obj(Json.parseObject("{\"x\":[]}"), "x"));
    }
}
