package boobuzz.core.hal;

/** Hicbir sey yapmaz. Yarismada bu kosar. */
public final class NullProbe implements Probe {

    public static final NullProbe INSTANCE = new NullProbe();

    private NullProbe() {}

    @Override
    public <T> void publish(String name, T message, long t) {
        // kasten bos
    }
}
