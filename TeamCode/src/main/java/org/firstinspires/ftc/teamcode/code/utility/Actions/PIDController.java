package org.firstinspires.ftc.teamcode.code.utility.Actions;

public class PIDController {
    private double kp;
    private double ki;
    private double kd;

    private double integral = 0.0;
    private double lastError;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double calculate(double error) {
        return p(error) + i(error) + d(error);
    }

    private double p(double error) {
        return error * kp;
    }

    private double i(double error) {
        integral += error * ki;
        return integral;
    }

    private double d(double error) {
        double derivative = (error - lastError) * kd;
        lastError = error;
        return derivative;
    }
}