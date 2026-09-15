package boobuzz.core.controller;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;

/**
 * L3. {@code decide(Feedback) -> Intent} ayni zamanda {@code policy(obs) -> action}
 * imzasidir; RLController bu yuzden ayri bir yol degil, ayni arayuzun baska bir
 * implementasyonudur.
 */
@FunctionalInterface
public interface Controller {
    Intent decide(Feedback feedback);
}
