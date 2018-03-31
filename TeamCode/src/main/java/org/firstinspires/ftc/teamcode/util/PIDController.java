package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Self-sufficient PID Controller
 */
public final class PIDController {
    // K-values
    private final double kProportional;
    private final double kIntegral;
    private final double kDerivative;

    private final double min;
    private final double max;

    private boolean firstTime = true;
    private double errSum = 0;
    private final ElapsedTime timer = new ElapsedTime();

    private double lastTime = 0;
    private double lastErr = 0;

    private double clamp(double val) {
        if (val < min) return min;
        if (val > max) return max;
        return val;
    }

    public PIDController(double min, double max, double kP, double kI, double kD) {
        kProportional = kP;
        kIntegral = kI;
        kDerivative = kD;

        this.min = min;
        this.max = max;
    }

    public PIDController(double kP, double kI, double kD) {
        this(-1, 1, kP, kI, kD);
    }

    public double get(double error) {
        double prop = kProportional * error;

        double inte = 0, deri = 0;
        if (!firstTime) {
            double dt = timer.seconds() - lastTime;
            lastTime = timer.seconds();

            // Integral
            errSum += dt * error;
            inte = errSum * kIntegral;

            // Derivative
            deri = (error - lastErr) / dt * kDerivative;
        }

        lastErr = error;
        if (firstTime) {
            timer.reset();
            firstTime = false;
        }

        return clamp(prop + inte + deri);
    }
}
