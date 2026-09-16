package boobuzz.core.controller.teleop;

import boobuzz.core.controller.Buttons;
import boobuzz.core.controller.IController;
import boobuzz.core.controller.auto.SequenceRunner;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.GamepadState;
import boobuzz.core.contract.IGamepadSource;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStream;

import java.util.ArrayList;
import java.util.List;

/** L3 controller: samples the gamepad, applies {@link TeleopMap}, and runs Y autos. */
public final class TeleopController implements IController {

    private final IGamepadSource gamepads;
    private final TeleopMap map;
    private Buttons buttons;
    private SequenceRunner sequenceRunner;
    private long lastTimeMs;

    public TeleopController(IGamepadSource gamepads) {
        this(gamepads, new TeleopMap());
    }

    public TeleopController(IGamepadSource gamepads, TeleopMap map) {
        this.gamepads = gamepads;
        this.map = map;
    }

    @Override
    public RequestBatch decide(Feedback feedback) {
        GamepadState current = gamepads.get();
        if (current == null) {
            return RequestBatch.idle();
        }
        long now = feedback == null ? lastTimeMs + 20L : feedback.t();
        if (buttons == null) {
            buttons = new Buttons(GamepadState.neutral(), current, lastTimeMs, now);
        } else {
            buttons.update(current, now);
        }
        lastTimeMs = now;

        TeleopMap.Mapping mapped = map.map(buttons, feedback);
        if (mapped.sequenceToStart() != null
                && (sequenceRunner == null || sequenceRunner.isFinished())) {
            sequenceRunner = new SequenceRunner(mapped.sequenceToStart());
        }

        RequestBatch direct = mapped.batch();
        if (sequenceRunner == null || sequenceRunner.isFinished()) {
            return direct;
        }

        // A real stick sample always wins over a background fluent sequence.
        if (direct.stream().manualDrive()) {
            int[] sequenceIds = sequenceRunner.activeRequestIds();
            sequenceRunner.abort("overridden by manual drive");
            // Driver takeover cancels every request owned by the sequence, including
            // attached shooter and intake work; unrelated manual requests continue.
            return withAdditionalCancels(direct, sequenceIds);
        }

        RequestBatch sequenceBatch = sequenceRunner.decide(feedback);
        return merge(sequenceBatch, direct);
    }

    public SequenceRunner sequenceRunner() {
        return sequenceRunner;
    }

    private static RequestBatch merge(RequestBatch sequence, RequestBatch mapped) {
        List<Request> requests = new ArrayList<>(sequence.requests().size()
                + mapped.requests().size());
        requests.addAll(sequence.requests());
        requests.addAll(mapped.requests());
        int[] sequenceCancels = sequence.cancels();
        int[] mappedCancels = mapped.cancels();
        int[] cancels = new int[sequenceCancels.length + mappedCancels.length];
        System.arraycopy(sequenceCancels, 0, cancels, 0, sequenceCancels.length);
        System.arraycopy(mappedCancels, 0, cancels, sequenceCancels.length,
                mappedCancels.length);
        RequestStream stream = sequence.stream().manualDrive()
                ? sequence.stream() : mapped.stream();
        return new RequestBatch(stream, requests, cancels);
    }

    private static RequestBatch withAdditionalCancels(RequestBatch batch, int[] additional) {
        int[] existing = batch.cancels();
        int[] cancels = new int[existing.length + additional.length];
        System.arraycopy(existing, 0, cancels, 0, existing.length);
        System.arraycopy(additional, 0, cancels, existing.length, additional.length);
        return new RequestBatch(batch.stream(), batch.requests(), cancels);
    }
}
