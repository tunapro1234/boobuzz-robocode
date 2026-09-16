package boobuzz.core.controller;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.RequestBatch;

/**
 * L3. {@code decide(Feedback) -> RequestBatch} is also the {@code policy(obs) -> action}
 * signature; RLController is therefore another implementation of the same
 * interface, not a separate path.
 */
@FunctionalInterface
public interface IController {
    RequestBatch decide(Feedback feedback);
}
