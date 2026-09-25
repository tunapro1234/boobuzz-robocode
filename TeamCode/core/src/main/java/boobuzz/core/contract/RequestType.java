package boobuzz.core.contract;

/** Edge-triggered request types. */
public enum RequestType {
    SHOOT,
    INTAKE,
    GOTO,
    PATH,
    SPIN_UP,
    INTAKE_ON,
    INTAKE_OFF,
    TURN_TO,
    TURRET_AIM,
    RESET_POSE,
    SWITCH_ENGINE,
    /** Phase-1.1-a addendum: [rpm, hoodDeg, turretRad]. */
    SET_SHOT_PRESET,
    /** Phase-1.1-a addendum: finish the current pulse, start no new ones. */
    STOP_SHOOTING,
    /** Phase-1.1-a addendum: [mode] 0 exit, 1 reverse intake/feeder, 2 held jam clear. */
    MECHANISM_RECOVERY
}
