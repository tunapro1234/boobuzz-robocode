package boobuzz.core.contract;

import java.util.Objects;

/** Immutable edge-triggered request sent from a controller to an engine. */
public record Request(int id, RequestType type, double[] params, PathRequest path) {

    public Request {
        type = Objects.requireNonNull(type, "type");
        params = params == null ? new double[0] : params.clone();
        if (type == RequestType.PATH && path == null) {
            throw new IllegalArgumentException("PATH request needs a path payload");
        }
        if (type != RequestType.PATH && path != null) {
            throw new IllegalArgumentException("only PATH requests may carry a path payload");
        }
    }

    public Request(int id, RequestType type, double... params) {
        this(id, type, params, null);
    }

    public Request(int id, RequestType type, PathRequest path) {
        this(id, type, new double[0], path);
    }

    public Request(int id, PathRequest path) {
        this(id, RequestType.PATH, path);
    }

    @Override
    public double[] params() {
        return params.clone();
    }

    public static Request of(int id, RequestType type, double... params) {
        return new Request(id, type, params);
    }

    public static Request path(int id, PathRequest path) {
        return new Request(id, path);
    }

    public static Request turnTo(int id, double headingRad) {
        return of(id, RequestType.TURN_TO, headingRad);
    }

    public static Request shoot(int id, int count) {
        return of(id, RequestType.SHOOT, count);
    }

    public static Request shoot(int id, int count, double rpm) {
        return of(id, RequestType.SHOOT, count, rpm);
    }

    public static Request spinUp(int id, double rpm) {
        return of(id, RequestType.SPIN_UP, rpm);
    }

    public static Request intakeOn(int id, double power) {
        return of(id, RequestType.INTAKE_ON, power);
    }

    public static Request intakeOff(int id) {
        return of(id, RequestType.INTAKE_OFF);
    }

    public double param(int index, double fallback) {
        return index >= 0 && index < params.length ? params[index] : fallback;
    }
}
