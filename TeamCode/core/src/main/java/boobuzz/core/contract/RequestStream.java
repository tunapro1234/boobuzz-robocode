package boobuzz.core.contract;

/** Per-tick level input from a controller; it has no id or completion status. */
public record RequestStream(double vx, double vy, double omega, boolean manualDrive) {

    public static RequestStream idle() {
        return new RequestStream(0.0, 0.0, 0.0, false);
    }

    public static RequestStream manual(double vx, double vy, double omega) {
        return new RequestStream(vx, vy, omega, true);
    }
}
