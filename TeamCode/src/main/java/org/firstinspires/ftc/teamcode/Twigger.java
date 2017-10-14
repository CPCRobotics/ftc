package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

/**
 * Twigger - helper class that combines both Telemetry and RobotLog
 *
 * Alternative to the "Logger".
 *
 * @see Telemetry
 * @see RobotLog
 *
 *
 */

public class Twigger {
    private static final Twigger ourInstance = new Twigger();

    private Telemetry telemetry;
    private final HashMap<String, Func<String>> dataFuncs = new HashMap<>();

    public static Twigger getInstance() {
        return ourInstance;
    }

    private Twigger() {
    }

    public class Line implements Func<String> {

        // Line should only be created by Twigger.
        private Line() {}

        Telemetry.Line line = telemetry.addLine();
        private final HashMap<String, Func<String>> dataFuncs = new HashMap<>();

        @Override
        public String value() {
            StringBuilder sb = new StringBuilder();

            for (String s : dataFuncs.keySet())
                sb.append(String.format("%s: %s | ", s, dataFuncs.get(s)));

            return sb.toString();
        }

        public Line addData(String name, final Object data) {
            return addData(name, new Func<String>() {
                @Override
                public String value() {
                    return data.toString();
                }
            });
        }

        public Line addData(String name, Func<String> data) {
            line.addData(name, data);
            dataFuncs.put(name, data);
            return this;
        }
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Line addLine() {
        return addLine("Line");
    }

    public Line addLine(String name) {
        Line line = new Line();
        dataFuncs.put(name, line);

        return line;
    }

    public Twigger addData(String name, final Object data) {
        return addData(name, new Func<String>() {
            @Override
            public String value() {
                return data.toString();
            }
        });
    }

    public Twigger addData(String name, Func<String> data) {
        telemetry.addData(name, data);
        dataFuncs.put(name, data);
        return this;
    }

    public void update() {
        telemetry.update();
        for (String n : dataFuncs.keySet())
            RobotLog.d("%s: %s", n, dataFuncs.get(n).value());
    }
}
