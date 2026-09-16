package boobuzz.core.controller;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;

/**
 * L3. {@code decide(Feedback) -> Intent} is also the {@code policy(obs) -> action}
 * signature; RLController is therefore another implementation of the same
 * interface, not a separate path.
 */
@FunctionalInterface
public interface IController {
    Intent decide(Feedback feedback);
}
