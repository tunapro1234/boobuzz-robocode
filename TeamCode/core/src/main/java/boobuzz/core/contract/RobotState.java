package boobuzz.core.contract;

import com.pedropathing.math.Pose;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * UPWARD raw sensor reading. Matches the protocol documentation exactly.
 *
 * <p>{@code t} is the simulator/robot clock and comes from the HAL time source
 * (constitution rule 2: time comes from HAL).
 *
 * <p>The simulator's {@code truth} field does NOT ENTER HERE - :core never sees
 * ground truth, otherwise code could work in simulation but fail on the robot.
 *
 * @param t       milliseconds, HAL clock
 * @param enc     motor name -> encoder ticks (integer)
 * @param vel     motor name -> ticks/second
 * @param yaw     IMU yaw, RADIANS
 * @param pinpoint odometry pose (inches, radians)
 * @param voltage bus voltage
 * @param analogVolts analog input name -> volts sampled at {@code t}; a missing name is
 *                    an invalid reading, never 0 V
 */
public record RobotState(long t,
                         Map<String, Integer> enc,
                         Map<String, Double> vel,
                         double yaw,
                         Pose pinpoint,
                         double voltage,
                         Map<String, Double> analogVolts) {

    public RobotState {
        enc = Collections.unmodifiableMap(new LinkedHashMap<>(enc));
        vel = Collections.unmodifiableMap(new LinkedHashMap<>(vel));
        analogVolts = Collections.unmodifiableMap(new LinkedHashMap<>(analogVolts));
    }

    /** Compatibility constructor for states without analog inputs (tests, protocol v2). */
    public RobotState(long t, Map<String, Integer> enc, Map<String, Double> vel, double yaw,
                      Pose pinpoint, double voltage) {
        this(t, enc, vel, yaw, pinpoint, voltage, Collections.emptyMap());
    }

}
