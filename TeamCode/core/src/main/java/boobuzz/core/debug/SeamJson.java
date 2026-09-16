package boobuzz.core.debug;

import boobuzz.core.contract.Event;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.WorldSnapshot;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Canonical JSON shape for the three R4 seam records and request batches. */
public final class SeamJson {

    private SeamJson() {}

    public static String hal(long tMs, RobotState state, RobotAction action) {
        Map<String, Object> root = root("hal", tMs);
        Map<String, Object> sensor = new LinkedHashMap<>();
        sensor.put("enc", state.enc());
        sensor.put("vel", state.vel());
        sensor.put("yaw", state.yaw());
        sensor.put("pinpoint", pose(state.pinpoint()));
        sensor.put("voltage", state.voltage());
        root.put("state", sensor);
        root.put("action", action(action));
        return JsonCodec.stringify(root);
    }

    public static String subsystem(long tMs, List<SubsystemTrace.Call> calls,
                                   List<Event> events) {
        Map<String, Object> root = root("subsystem", tMs);
        root.put("calls", calls == null ? List.of() : calls.stream()
                .map(SeamJson::call).toList());
        root.put("events", events == null ? List.of() : events.stream()
                .map(SeamJson::event).toList());
        return JsonCodec.stringify(root);
    }

    public static String logic(long tMs, Feedback feedback, RequestBatch batch) {
        Map<String, Object> root = root("logic", tMs);
        root.put("feedback", feedback(feedback));
        root.put("batch", batchMap(batch));
        return JsonCodec.stringify(root);
    }

    public static Map<String, Object> batchMap(RequestBatch batch) {
        batch = batch == null ? RequestBatch.idle() : batch;
        Map<String, Object> result = new LinkedHashMap<>();
        RequestStream stream = batch.stream();
        Map<String, Object> streamMap = new LinkedHashMap<>();
        streamMap.put("vx", stream.vx());
        streamMap.put("vy", stream.vy());
        streamMap.put("omega", stream.omega());
        streamMap.put("manualDrive", stream.manualDrive());
        result.put("stream", streamMap);
        result.put("requests", batch.requests().stream().map(SeamJson::request).toList());
        List<Integer> cancels = new ArrayList<>();
        for (int cancel : batch.cancels()) cancels.add(cancel);
        result.put("cancels", cancels);
        return result;
    }

    /** Converts values used in subsystem call arguments into JSON-safe values. */
    public static Object value(Object value) {
        if (value == null || value instanceof String || value instanceof Number
                || value instanceof Boolean) {
            return value;
        }
        if (value instanceof Pose poseValue) return pose(poseValue);
        if (value instanceof PathRequest pathValue) return path(pathValue);
        if (value instanceof Iterable<?> iterable) {
            return java.util.stream.StreamSupport.stream(iterable.spliterator(), false)
                    .map(SeamJson::value).toList();
        }
        if (value instanceof Map<?, ?> map) {
            Map<String, Object> result = new LinkedHashMap<>();
            for (Map.Entry<?, ?> entry : map.entrySet()) {
                result.put(String.valueOf(entry.getKey()), value(entry.getValue()));
            }
            return result;
        }
        return String.valueOf(value);
    }

    public static RequestBatch batchFrom(Map<String, Object> root) {
        Map<String, Object> batch = root.containsKey("batch")
                ? JsonCodec.object(root, "batch") : root;
        Map<String, Object> stream = JsonCodec.object(batch, "stream");
        RequestStream requestStream = new RequestStream(
                JsonCodec.num(stream, "vx", 0.0),
                JsonCodec.num(stream, "vy", 0.0),
                JsonCodec.num(stream, "omega", 0.0),
                JsonCodec.bool(stream, "manualDrive", false));
        List<Request> requests = new ArrayList<>();
        for (Object value : JsonCodec.list(batch, "requests")) {
            if (value instanceof Map<?, ?> map) {
                @SuppressWarnings("unchecked")
                Map<String, Object> request = (Map<String, Object>) map;
                requests.add(request(request));
            }
        }
        List<Integer> cancels = new ArrayList<>();
        for (Object value : JsonCodec.list(batch, "cancels")) {
            if (value instanceof Number number) cancels.add(number.intValue());
        }
        int[] cancelArray = cancels.stream().mapToInt(Integer::intValue).toArray();
        return new RequestBatch(requestStream, requests, cancelArray);
    }

    private static Map<String, Object> root(String seam, long tMs) {
        Map<String, Object> root = new LinkedHashMap<>();
        root.put("seam", seam);
        root.put("t_ms", tMs);
        return root;
    }

    private static Map<String, Object> action(RobotAction value) {
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("motors", value.motors());
        result.put("servos", value.servos());
        result.put("events", value.events().stream().map(SeamJson::event).toList());
        return result;
    }

    private static Map<String, Object> call(SubsystemTrace.Call value) {
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("sub", value.sub());
        result.put("op", value.op());
        result.put("args", value.args().stream().map(SeamJson::value).toList());
        return result;
    }

    private static Map<String, Object> event(Event value) {
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("name", value.name());
        result.put("t_ms", value.tMs());
        result.put("data", value.data());
        return result;
    }

    private static Map<String, Object> feedback(Feedback value) {
        Map<String, Object> result = new LinkedHashMap<>();
        if (value == null) {
            result.put("snapshot", Map.of());
            result.put("statuses", List.of());
            return result;
        }
        result.put("snapshot", snapshot(value.world()));
        result.put("statuses", value.statuses().stream().map(SeamJson::status).toList());
        return result;
    }

    private static Map<String, Object> snapshot(WorldSnapshot value) {
        Map<String, Object> result = new LinkedHashMap<>();
        if (value == null) return result;
        result.put("t_ms", value.t());
        result.put("pose", pose(value.pose()));
        result.put("yaw", value.yaw());
        result.put("voltage", value.voltage());
        return result;
    }

    private static Map<String, Object> status(RequestStatus value) {
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("id", value.id());
        result.put("state", value.state().name());
        result.put("progress", value.progress());
        result.put("note", value.note());
        return result;
    }

    private static Map<String, Object> request(Request value) {
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("id", value.id());
        result.put("type", value.type().name());
        List<Double> params = new ArrayList<>();
        for (double param : value.params()) params.add(param);
        result.put("params", params);
        result.put("path", value.path() == null ? null : path(value.path()));
        return result;
    }

    private static Map<String, Object> path(PathRequest value) {
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("pathId", value.pathId());
        result.put("target", pose(value.target()));
        result.put("constraints", Map.of("maxPower", value.constraints().maxPower(),
                "maxVelocity", value.constraints().maxVelocity()));
        result.put("segments", value.segments().stream().map(SeamJson::segment).toList());
        result.put("heading", Map.of("mode", value.heading().mode().name(),
                "start", value.heading().start(), "end", value.heading().end()));
        result.put("holdEnd", value.holdEnd());
        result.put("velocityConstraint", value.velocityConstraint());
        result.put("braking", value.braking() == null ? null : Map.of(
                "strength", value.braking().strength(),
                "startMultiplier", value.braking().startMultiplier()));
        return result;
    }

    private static Map<String, Object> segment(PathRequest.Segment value) {
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("kind", value instanceof PathRequest.Curve ? "curve" : "line");
        result.put("end", pose(value.end()));
        if (value instanceof PathRequest.Curve curve) {
            result.put("controlPoints", curve.controlPoints().stream()
                    .map(SeamJson::pose).toList());
        }
        return result;
    }

    private static Map<String, Object> pose(Pose value) {
        if (value == null) return null;
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("x", value.x());
        result.put("y", value.y());
        result.put("h", value.heading());
        return result;
    }

    private static Request request(Map<String, Object> value) {
        int id = (int) JsonCodec.num(value, "id", 0);
        boobuzz.core.contract.RequestType type = boobuzz.core.contract.RequestType.valueOf(
                JsonCodec.str(value, "type", "INTAKE"));
        List<Object> params = JsonCodec.list(value, "params");
        double[] numbers = new double[params.size()];
        for (int i = 0; i < params.size(); i++) {
            numbers[i] = params.get(i) instanceof Number n ? n.doubleValue() : 0.0;
        }
        Map<String, Object> pathMap = JsonCodec.object(value, "path");
        if (pathMap.isEmpty() && value.get("path") == null) {
            return new Request(id, type, numbers);
        }
        return new Request(id, type, pathFrom(pathMap));
    }

    private static PathRequest pathFrom(Map<String, Object> value) {
        String pathId = value.get("pathId") == null ? null
                : JsonCodec.str(value, "pathId", null);
        Pose target = poseFrom(JsonCodec.object(value, "target"));
        Map<String, Object> constraints = JsonCodec.object(value, "constraints");
        PathRequest.Constraints limits = new PathRequest.Constraints(
                JsonCodec.num(constraints, "maxPower", 1.0),
                JsonCodec.num(constraints, "maxVelocity", Double.MAX_VALUE));
        Map<String, Object> heading = JsonCodec.object(value, "heading");
        PathRequest.Heading pathHeading = new PathRequest.Heading(
                PathRequest.HeadingMode.valueOf(JsonCodec.str(heading, "mode", "TANGENT")),
                JsonCodec.num(heading, "start", 0.0),
                JsonCodec.num(heading, "end", 0.0));
        List<PathRequest.Segment> segments = new ArrayList<>();
        for (Object item : JsonCodec.list(value, "segments")) {
            if (!(item instanceof Map<?, ?> raw)) continue;
            @SuppressWarnings("unchecked") Map<String, Object> segment = (Map<String, Object>) raw;
            Pose end = poseFrom(JsonCodec.object(segment, "end"));
            if ("curve".equals(JsonCodec.str(segment, "kind", "line"))) {
                List<Pose> controls = new ArrayList<>();
                for (Object control : JsonCodec.list(segment, "controlPoints")) {
                    if (control instanceof Map<?, ?> rawControl) {
                        @SuppressWarnings("unchecked") Map<String, Object> controlMap =
                                (Map<String, Object>) rawControl;
                        controls.add(poseFrom(controlMap));
                    }
                }
                segments.add(new PathRequest.Curve(end, controls));
            } else {
                segments.add(new PathRequest.Line(end));
            }
        }
        Double velocity = value.get("velocityConstraint") instanceof Number n
                ? n.doubleValue() : null;
        Map<String, Object> braking = JsonCodec.object(value, "braking");
        PathRequest.Braking brake = braking.isEmpty() ? null : new PathRequest.Braking(
                JsonCodec.num(braking, "strength", 0.0),
                JsonCodec.num(braking, "startMultiplier", 0.0));
        return new PathRequest(pathId, target, limits, segments, pathHeading,
                JsonCodec.bool(value, "holdEnd", true), velocity, brake);
    }

    private static Pose poseFrom(Map<String, Object> value) {
        return value.isEmpty() ? null : new Pose(
                JsonCodec.num(value, "x", 0.0),
                JsonCodec.num(value, "y", 0.0),
                JsonCodec.num(value, "h", 0.0));
    }
}
