package boobuzz.core.logic;

import boobuzz.core.contract.Intent;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;

/**
 * L2 icinde iki yonlu subsystem siniri.
 *
 * <p>{@link #observe} yukari akan ham durumu, {@link #update} asagi akan niyeti
 * alir. {@link RobotAction} immutable oldugu icin subsystem'ler ayni sirali
 * {@link RobotAction.Builder} uzerine yazar; son eylemi engine tek kez dondurur.
 */
public interface Subsystem {

    String name();

    void observe(long now, RobotState state);

    void update(Intent intent, RobotAction.Builder out);
}
