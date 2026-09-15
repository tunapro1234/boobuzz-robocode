package boobuzz.core.logic.engine;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;

/**
 * L2. Iki yonlu: {@link #sense} yukari algi, {@link #act} asagi icra.
 * Tamami takilip cikarilabilir bir birimdir.
 *
 * <pre>{@code RobotEngine engine = new C3VisionEngine(hal);  // C2'ye donmek TEK SATIR}</pre>
 *
 * <p>Amac bu: world model bozulursa C3'e, vision bozulursa C2'ye, her sey bozulursa
 * C1'e donersin - yarisma sabahi, tek satirla. Gecen sezon boyle bir geri donus yoktu.
 */
public interface RobotEngine {

    String name();

    /** YUKARI: ham durumdan dunya gorusu + istek durumlari. */
    Feedback sense(long now, RobotState state);

    /** ASAGI: niyetten motor/servo komutu. */
    RobotAction act(Intent intent);
}
