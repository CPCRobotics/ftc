package cpc.robotics.logging;

import android.util.Log;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.webserver.SessionCookie;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;

import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TimeZone;
import java.util.concurrent.ConcurrentHashMap;

import fi.iki.elonen.NanoHTTPD;

public class LogServer extends NanoHTTPD {

    private static final String TAG = "LogServer";
    private final static String MARKER_NAME = "marker"; // named parameter for log marker
    private final static String ROOT_URI = "/"; // accessing root directory
    private final static String LOG_URI = "/log"; // regular log
    private final static String TELEMETRY_URI = "/telemetry"; // telemetry data
    private final static String MIME_JSON = "application/json";


    private static long INDEX_SPAN = 1000L; // cache maximum 1000 log lines
    public static final int PORT = 1234;
    public static final LogServer singleton = new LogServer();
    private Object logSyncObject = new Object();
    private Object telemSyncObject = new Object();
    private LinkedList<LogEntry> logQueue = new LinkedList<>();
    private long logIndex = 0;
    private LinkedHashMap<String, LogEntry> telemetry = new LinkedHashMap<>();
    private volatile boolean started = false;
    private final ConcurrentHashMap<String, WebHandler> handlerMap = new ConcurrentHashMap<>();
    public static String staticDateStamp;
    public static SimpleDateFormat gmtFrmt = new SimpleDateFormat("E, d MMM yyyy HH:mm:ss 'PST'", Locale.US);
    static {
        gmtFrmt.setTimeZone(TimeZone.getTimeZone("PST"));
        staticDateStamp = gmtFrmt.format(new Date());
    }

    /**
     * Add and/or update telemetry data.
     * @param tag Name of telemetry data to add
     * @param data New data. Objects must be immutable (String, LogEntry, etc)
     */
    public void updateTelemetry(String tag, Object data) {
        LogEntry entry = new LogEntry(new Date(), tag, data);
        synchronized (telemSyncObject) {
            telemetry.put(tag, entry);
        }
    }

    /**
     * Remove telemetry setting - note, doing this may change order of telemetry data
     * @param tag telemetry data to remove
     */
    public void removeTelemetry(String tag) {
        synchronized (telemSyncObject) {
            telemetry.remove(tag);
        }
    }

    /**
     * Remove all telemetry. New telemetry may change order when written, but in general,
     * this is fine to call.
     */
    public void resetTelemetry() {
        synchronized (telemSyncObject) {
            telemetry.clear();
        }
    }

    /**
     * This is used to capture a log entry, and also to serialize to JSON. Object is immutable,
     * so it is acceptable to have LogEntry values passed to LogEntry if desired.
     */
    public static class LogEntry {
        public final Date time; // cannot be null
        public final String tag; // cannot be null
        public final Object data; // expected to be immutable, may be null
        public LogEntry(Date time, String tag, Object data) {
            this.time = time;
            this.tag = tag;
            this.data = data;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;

            LogEntry logEntry = (LogEntry) o;

            if (!time.equals(logEntry.time)) return false;
            if (!tag.equals(logEntry.tag)) return false;
            return data != null ? data.equals(logEntry.data) : logEntry.data == null;
        }

        @Override
        public int hashCode() {
            int result = time.hashCode();
            result = 31 * result + tag.hashCode();
            result = 31 * result + (data != null ? data.hashCode() : 0);
            return result;
        }
    }

    /**
     * This will serialize to JSON, used for a simple sorted collection
     */
    public static class SimpleCollection {
        public LogEntry [] entries;
        public SimpleCollection(LogEntry [] entries) {
            this.entries = entries;
        }
    }

    /**
     * This will serialize to JSON, specifying start/marker
     */
    public static class MarkedCollection extends SimpleCollection {
        public final long start;
        public final long marker;
        public MarkedCollection(long start, long marker, LogEntry [] entries) {
            super(entries);
            this.start = start;
            this.marker = marker;
        }
    }

    public LogServer() {
        super(PORT);
        handlerMap.put(LOG_URI, new LogUriHandler());
        handlerMap.put(TELEMETRY_URI, new TelemetryUriHandler());
    }

    public void start() {
        if (!this.started) {
            try {
                super.start();
                Log.i(TAG, "Started Http Server on Port " + String.valueOf(PORT));
            } catch (IOException e) {
                e.printStackTrace();
                Log.e(TAG,"Unable to start Http LogServer");
            }
        }
    }

    public void write(String tag, String text) {
        Date now = new Date();
        synchronized(logSyncObject) {
            // Simple synchronization should be sufficient as there should be little
            // contention
            ++logIndex;
            LogEntry entry = new LogEntry(now, tag, text);
            logQueue.add(entry);
            while (logQueue.size() > INDEX_SPAN) {
                logQueue.remove();
            }
        }
    }

    @Override
    public Response serve(IHTTPSession session) {
        SessionCookie.ensureInSession(session);

        final String command = session.getUri();
        final WebHandler webHandler = handlerMap.get(command);

        try {
            if (webHandler == null) {
                NanoHTTPD.Response response = newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND, NanoHTTPD.MIME_PLAINTEXT, "");
                response.addHeader("Date", staticDateStamp);
                return response;
            } else {
                return webHandler.getResponse(session);
            }
        } catch (IOException e) {
            RobotLog.logStackTrace(e);
            return newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR, NanoHTTPD.MIME_PLAINTEXT, "Internal Error");
        } catch (NanoHTTPD.ResponseException e) {
            RobotLog.logStackTrace(e);
            return newFixedLengthResponse(e.getStatus(), NanoHTTPD.MIME_PLAINTEXT, e.getMessage());
        } catch (RuntimeException e) {
            return newFixedLengthResponse(Response.Status.INTERNAL_ERROR, NanoHTTPD.MIME_PLAINTEXT, e.getMessage());
        }
    }

    protected static String getFirstNamedParameter(IHTTPSession session, String name) {
        final Map<String, List<String>> parameters = session.getParameters();
        if (!parameters.containsKey(name)) {
            return null;
        }
        return parameters.get(name).get(0);
    }
    protected static Long getIntegerParameter(IHTTPSession session, String name) {
        String value = getFirstNamedParameter(session, name);
        if (value == null || value.length() == 0) {
            return null;
        } else {
            return Long.valueOf(value);
        }
    }

    static private Gson newGson() {
        return new GsonBuilder().
                serializeNulls().
                setDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'").
                setPrettyPrinting().create();
    }

    protected class LogUriHandler implements WebHandler {

        @Override
        public Response getResponse(IHTTPSession session) throws IOException, ResponseException {
            Long lastMarkerRaw = getIntegerParameter(session, MARKER_NAME);
            // lastMarker allows optimization of what data is returned to caller
            long lastMarker = (lastMarkerRaw == null) ? 0 : lastMarkerRaw;
            LogEntry [] logSnapshot;
            long baseIndex;
            long markerSnapshot;
            synchronized(logSyncObject) {
                // sit in here as little as possible, as it may pause the attempt to add to log
                // creating a snapshot is fastest approach
                // Each logEntry is immutable so can be shared
                // Everything else can be derived from logQueue snapshot and logIndex snapshot
                logSnapshot = logQueue.toArray(new LogEntry[logQueue.size()]);
                markerSnapshot = logIndex;
            }
            baseIndex = markerSnapshot - logSnapshot.length;
            if (lastMarker > markerSnapshot || lastMarker < baseIndex) {
                // In first case, impossible marker, maybe process was reset?
                // In second case, log entries were thrown away
                lastMarker = baseIndex;
            }
            Gson gson = new GsonBuilder().serializeNulls().setPrettyPrinting().create();
            int start = 0;
            if (baseIndex < lastMarker) {
                // can truncate log
                start = (int)(lastMarker - baseIndex);
            }
            MarkedCollection collection = new MarkedCollection(lastMarker, markerSnapshot, Arrays.copyOfRange(logSnapshot, start, logSnapshot.length));
            String json = newGson().toJson(collection);
            return newFixedLengthResponse(NanoHTTPD.Response.Status.OK, MIME_JSON, json);
        }
    }

    protected class TelemetryUriHandler implements WebHandler {

        @Override
        public Response getResponse(IHTTPSession session) throws IOException, ResponseException {
            LogEntry [] logSnapshot;
            synchronized(telemSyncObject) {
                // sit in here as little as possible, as it may pause the attempt to add to log
                // creating a snapshot is fastest approach
                // Each logEntry is immutable so can be shared
                // Everything else can be derived from logQueue snapshot and logIndex snapshot
                logSnapshot = telemetry.values().toArray(new LogEntry[telemetry.size()]);
            }
            String json = newGson().toJson(new SimpleCollection(logSnapshot));
            return newFixedLengthResponse(NanoHTTPD.Response.Status.OK, MIME_JSON, json);
        }
    }
}
