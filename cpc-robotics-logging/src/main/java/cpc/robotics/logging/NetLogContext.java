package cpc.robotics.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NetLogContext {

    private final String tag;

    public NetLogContext(Class<?> cls) {
        this(cls.getSimpleName());
    }

    public NetLogContext(String tag) {
        if (tag == null) {
            throw new NullPointerException("Tag cannot be null");
        }
        this.tag = tag;
        // Start the server when the first context is constructed
        LogServer.singleton.start();
    }

    public void write(String text) {
        LogServer.singleton.write(tag, text);
    }

    public void updateTelemetry(String tag, Object data) {
        LogServer.singleton.updateTelemetry(tag, data);
    }

    public void removeTelemetry(String tag) {
        LogServer.singleton.removeTelemetry(tag);
    }

    public void resetTelemetry() {
        LogServer.singleton.resetTelemetry();
    }
}
