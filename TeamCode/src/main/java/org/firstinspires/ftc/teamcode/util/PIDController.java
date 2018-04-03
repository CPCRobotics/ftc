package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

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

    private final transient double speed;

    private transient boolean firstTime = true;
    private transient double errSum = 0;
    private transient final ElapsedTime timer = new ElapsedTime();

    private transient double lastTime = 0;
    private transient double lastErr = 0;

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

    public double get(double error) {
        double prop = kP * error;

        double inte = 0, deri = 0;
        if (!firstTime) {
            double dt = timer.seconds() - lastTime;
            lastTime = timer.seconds();

            // Integral
            errSum += dt * error;
            inte = errSum * kI;

            // Derivative
            deri = (error - lastErr) / dt * kD;
        }

        lastErr = error;
        if (firstTime) {
            timer.reset();
            firstTime = false;
        }

        return clamp(prop + inte + deri);
    }

    /**
     * Save PID configuration to a JSON file
     */
    public void save(String filename) {
        File file = AppUtil.getInstance().getSettingsFile(filename);

        // Serialize it in JSON format
        String serializedJson = SimpleGson.getInstance().toJson(this);
        ReadWriteFile.writeFile(file, serializedJson);

        Twigger.getInstance().sendOnce(String.format("Saved to %s", filename));
    }

    /**
     * Load PID configuration from a JSON file w/ speed, and use default PID values
     * if unsuccessful
     *
     * @param filename name of configuration file
     * @param speed speed of controller
     * @param defKP default kP value
     * @param defKI default kI value
     * @param defKD default kD value
     * @return loaded PIDController, or new controller with default values if unsuccessful
     */
    public static PIDController load(String filename, double speed,
                                     double defKP, double defKI, double defKD) {
        File file = AppUtil.getInstance().getSettingsFile(filename);
        if (!file.exists()) {
            // Warning isn't a good sign
            Twigger.getInstance().sendOnce(String.format("WARN: PID config %s doesn't exist", filename));

            // Fall back to default PID values
            return new PIDController(speed, defKD, defKI, defKP);
        }

        String serializedJson = ReadWriteFile.readFile(file);
        PIDController fromMemory = SimpleGson.getInstance().fromJson(serializedJson, PIDController.class);

        // Show in telemetry
        Twigger.getInstance().sendOnce(String.format("Loaded into memory %s", fromMemory.toString()));
        return new PIDController(speed, fromMemory.kP, fromMemory.kI, fromMemory.kD);
    }

    /**
     * Show PID values
     */
    public String toString() {
        return String.format(Locale.ENGLISH,"(PID P=%f, I=%f, D=%f)", kP, kI, kD);
    }
}
