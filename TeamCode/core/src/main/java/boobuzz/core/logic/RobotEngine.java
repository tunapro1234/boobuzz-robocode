package boobuzz.core.logic;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;

/**
 * L2. Iki yonlu: {@link #sense} yukari algi, {@link #act} asagi icra.
 * Robot ve sim ayni engine uygulamasini bu arayuz uzerinden calistirir.
 */
public interface RobotEngine {

    String name();

    /** YUKARI: ham durumdan dunya gorusu + istek durumlari. */
    Feedback sense(long now, RobotState state);

    /** ASAGI: niyetten motor/servo komutu. */
    RobotAction act(Intent intent);
}
