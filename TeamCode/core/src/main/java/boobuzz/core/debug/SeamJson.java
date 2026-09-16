package boobuzz.core.debug;

import boobuzz.core.contract.Event;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RequestType;
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
                .map(SeamJson::call).collect(java.util.stream.Collectors.toList()));
        root.put("events", events == null ? List.of() : events.stream()
                .map(SeamJson::event).collect(java.util.stream.Collectors.toList()));
        return JsonCodec.stringify(root);
    }

    public static String logic(long tMs, Feedback feedback, RequestBatch batch) {
        Map<String, Object> root = root("logic", tMs);
        root.put("feedback", feedbackMap(feedback));
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
        result.put("requests", batch.requests().stream().map(SeamJson::request)
                .collect(java.util.stream.Collectors.toList()));
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
                    .map(SeamJson::value)
                    .collect(java.util.stream.Collectors.toList());
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
        validateBatch(batch);
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

    /** Rejects malformed controller batches before they can refresh a watchdog. */
    private static void validateBatch(Map<String, Object> batch) {
        if (batch == null || !batch.containsKey("stream")
                || !batch.containsKey("requests") || !batch.containsKey("cancels")) {
            throw new IllegalArgumentException(
                    "request batch requires stream, requests, and cancels");
        }
        Map<String, Object> stream = requiredObject(batch, "stream");
        finite(stream, "vx");
        finite(stream, "vy");
        finite(stream, "omega");
        if (!(stream.get("manualDrive") instanceof Boolean)) {
            throw new IllegalArgumentException("stream.manualDrive must be boolean");
        }
        List<Object> requests = requiredList(batch, "requests");
        for (Object item : requests) {
            if (!(item instanceof Map<?, ?> raw)) {
                throw new IllegalArgumentException("request must be an object");
            }
            @SuppressWarnings("unchecked") Map<String, Object> request =
                    (Map<String, Object>) raw;
            integral(request, "id");
            Object type = request.get("type");
            if (!(type instanceof String typeName)) {
                throw new IllegalArgumentException("request.type must be a string");
            }
            RequestType requestType;
            try {
                requestType = RequestType.valueOf(typeName);
            } catch (IllegalArgumentException e) {
                throw new IllegalArgumentException("unknown request type: " + typeName, e);
            }
            List<Object> params = requiredList(request, "params");
            for (Object param : params) {
                finite(param, "request parameter");
            }
            Object path = request.get("path");
            if (requestType == RequestType.PATH) {
                if (!(path instanceof Map<?, ?> rawPath)) {
                    throw new IllegalArgumentException("PATH request requires path object");
                }
                @SuppressWarnings("unchecked") Map<String, Object> pathMap =
                        (Map<String, Object>) rawPath;
                validatePath(pathMap);
            } else if (path != null) {
                throw new IllegalArgumentException("only PATH requests may carry a path");
            }
        }
        for (Object cancel : requiredList(batch, "cancels")) {
            integral(cancel, "cancel id");
        }
    }

    private static void validatePath(Map<String, Object> path) {
        Object pathId = path.get("pathId");
        if (pathId != null && (!(pathId instanceof String)
                || ((String) pathId).trim().isEmpty())) {
            throw new IllegalArgumentException("pathId must be a non-empty string or null");
        }
        Object target = path.get("target");
        if (target != null) validatePose(target, "target");
        Map<String, Object> constraints = requiredObject(path, "constraints");
        finite(constraints, "maxPower");
        finite(constraints, "maxVelocity");
        List<Object> segments = requiredList(path, "segments");
        for (Object item : segments) {
            if (!(item instanceof Map<?, ?> raw)) {
                throw new IllegalArgumentException("path segment must be an object");
            }
            @SuppressWarnings("unchecked") Map<String, Object> segment =
                    (Map<String, Object>) raw;
            Object kind = segment.get("kind");
            if (!(kind instanceof String)
                    || !("line".equals(kind) || "curve".equals(kind))) {
                throw new IllegalArgumentException("segment.kind must be line or curve");
            }
            validatePose(segment.get("end"), "segment.end");
            if ("curve".equals(kind)) {
                for (Object control : requiredList(segment, "controlPoints")) {
                    validatePose(control, "curve control point");
                }
            }
        }
        Map<String, Object> heading = requiredObject(path, "heading");
        Object mode = heading.get("mode");
        if (!(mode instanceof String modeName)) {
            throw new IllegalArgumentException("heading.mode must be a string");
        }
        try {
            PathRequest.HeadingMode.valueOf(modeName);
        } catch (IllegalArgumentException e) {
            throw new IllegalArgumentException("unknown heading mode: " + modeName, e);
        }
        finite(heading, "start");
        finite(heading, "end");
        if (!(path.get("holdEnd") instanceof Boolean)) {
            throw new IllegalArgumentException("path.holdEnd must be boolean");
        }
        Object velocity = path.get("velocityConstraint");
        if (velocity != null) finite(velocity, "velocityConstraint");
        Object braking = path.get("braking");
        if (braking != null) {
            Map<String, Object> brake = requiredObject(path, "braking");
            finite(brake, "strength");
            finite(brake, "startMultiplier");
        }
    }

    private static void validatePose(Object value, String label) {
        if (!(value instanceof Map<?, ?> raw)) {
            throw new IllegalArgumentException(label + " must be an object");
        }
        @SuppressWarnings("unchecked") Map<String, Object> pose = (Map<String, Object>) raw;
        finite(pose, "x");
        finite(pose, "y");
        finite(pose, "h");
    }

    private static Map<String, Object> requiredObject(Map<String, Object> object, String key) {
        Object value = object.get(key);
        if (!(value instanceof Map<?, ?> map)) {
            throw new IllegalArgumentException(key + " must be an object");
        }
        @SuppressWarnings("unchecked") Map<String, Object> result = (Map<String, Object>) map;
        return result;
    }

    private static List<Object> requiredList(Map<String, Object> object, String key) {
        Object value = object.get(key);
        if (!(value instanceof List<?> list)) {
            throw new IllegalArgumentException(key + " must be an array");
        }
        @SuppressWarnings("unchecked") List<Object> result = (List<Object>) list;
        return result;
    }

    private static void finite(Map<String, Object> object, String key) {
        if (!object.containsKey(key)) {
            throw new IllegalArgumentException(key + " is required");
        }
        finite(object.get(key), key);
    }

    private static void finite(Object value, String label) {
        if (!(value instanceof Number number) || value instanceof Boolean
                || !Double.isFinite(number.doubleValue())) {
            throw new IllegalArgumentException(label + " must be finite number");
        }
    }

    private static void integral(Map<String, Object> object, String key) {
        if (!object.containsKey(key)) {
            throw new IllegalArgumentException(key + " is required");
        }
        integral(object.get(key), key);
    }

    private static void integral(Object value, String label) {
        finite(value, label);
        double number = ((Number) value).doubleValue();
        if (number != Math.rint(number) || number < Integer.MIN_VALUE
                || number > Integer.MAX_VALUE) {
            throw new IllegalArgumentException(label + " must be a 32-bit integer");
        }
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
        result.put("events", value.events().stream().map(SeamJson::event)
                .collect(java.util.stream.Collectors.toList()));
        return result;
    }

    private static Map<String, Object> call(SubsystemTrace.Call value) {
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("sub", value.sub());
        result.put("op", value.op());
        result.put("args", value.args().stream().map(SeamJson::value)
                .collect(java.util.stream.Collectors.toList()));
        return result;
    }

    private static Map<String, Object> event(Event value) {
        Map<String, Object> result = new LinkedHashMap<>();
        result.put("name", value.name());
        result.put("t_ms", value.tMs());
        result.put("data", value.data());
        return result;
    }

    /** JSON-safe feedback map shared by the logic seam and SocketController. */
    public static Map<String, Object> feedbackMap(Feedback value) {
        Map<String, Object> result = new LinkedHashMap<>();
        if (value == null) {
            result.put("snapshot", Map.of());
            result.put("statuses", List.of());
            return result;
        }
        result.put("snapshot", snapshot(value.world()));
        result.put("statuses", value.statuses().stream().map(SeamJson::status)
                .collect(java.util.stream.Collectors.toList()));
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
        result.put("segments", value.segments().stream().map(SeamJson::segment)
                .collect(java.util.stream.Collectors.toList()));
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
                    .map(SeamJson::pose)
                    .collect(java.util.stream.Collectors.toList()));
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
