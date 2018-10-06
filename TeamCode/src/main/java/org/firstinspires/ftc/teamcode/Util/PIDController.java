package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/**
 * Self-sufficient PID Controller
 */
public final class PIDController {
    // K-values
    private final double kP;
    private final double kI;
    private final double kD;

    private final double speed;

    private boolean firstTime = true;
    private double errSum = 0;
    private final ElapsedTime timer = new ElapsedTime();

    private double lastTime = 0;
    private double lastErr = 0;

    public static final class PIDConfiguration {
        public final double kP;
        public final double kI;
        public final double kD;

        public PIDConfiguration(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public PIDConfiguration kP(double kP) {
            return new PIDConfiguration(kP, kI, kD);
        }

        public PIDConfiguration kI(double kI) {
            return new PIDConfiguration(kP, kI, kD);
        }

        public PIDConfiguration kD(double kD) {
            return new PIDConfiguration(kP, kI, kD);
        }

        public PIDController finish(double speed) {
            return new PIDController(speed, kP, kI, kD);
        }

        /**
         * Save PID configuration to a JSON file
         */
        public void save(String filename) {
            File file = AppUtil.getInstance().getSettingsFile(filename);

            // Serialize it in JSON format
            String serializedJson = SimpleGson.getInstance().toJson(this);
            ReadWriteFile.writeFile(file, serializedJson);
        }

        /**
         * Return configuration from file, or itself if unsuccessful
         * @param filename
         */
        public PIDConfiguration load(String filename) {
            File file = AppUtil.getInstance().getSettingsFile(filename);
            if (!file.exists()) {
                return this;
            }

            String json = ReadWriteFile.readFile(file);
            PIDConfiguration loadedConfig = SimpleGson.getInstance().fromJson(json, PIDConfiguration.class);
            return loadedConfig;
        }

        @Override
        public String toString() {
            return String.format(Locale.ENGLISH, "[PIDConfig P=%f, I=%f, D=%f]", kP, kI, kD);
        }
    }


    private double clamp(double val) {
        if (val < -speed) return -speed;
        if (val > speed) return speed;
        return val;
    }

    public PIDController(double speed, double kP, double kI, double kD) {
        this.speed = speed;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double get(double error)
    {
        double curTime = timer.seconds();

        double slope = 0;
        if (!firstTime)
        {
            double deltaT = curTime - lastTime;

            // Integral
            errSum += deltaT * error;

            // Derivative
            slope = (error - lastErr) / deltaT;
        }

        // Setting up for next iteration
        lastTime = curTime;
        lastErr = error;
        if (firstTime)
        {
            timer.reset();
            firstTime = false;
        }

        return clamp(error * kP + errSum * kI + slope * kD);
    }
}
