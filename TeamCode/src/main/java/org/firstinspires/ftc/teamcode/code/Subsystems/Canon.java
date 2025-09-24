package org.firstinspires.ftc.teamcode.code.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Set;

public class Canon {
    private DcMotorEx motor1;
    private DcMotorEx motor2;

    public Canon(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        //motor2 = hardwareMap.get(DcMotorEx.class, "outtakeR");
        //motor2 = motor1;
    }

    private class SpinUp implements Action {
        private double targetSpeed;
        private final double k = (1.0 / 2_500);

        public SpinUp(double target) {
            targetSpeed = target;
        }

        public boolean run(@NonNull TelemetryPacket t) {
            double speed = motor1.getVelocity();
            double error = targetSpeed - speed;
            error = Math.sqrt(Math.abs(error)) * (error > 0 ? 1 : -1);
            motor1.setPower(motor1.getPower() + (error*k));

            return Math.abs(error) > 20;
        }
    }
    private class MaintainSpeed extends SpinUp {
        public MaintainSpeed(double target) {
            super(target);
        }
        public boolean run(@NonNull TelemetryPacket t) {
            super.run(t);
            return true;
        }
    }

    public Action spinUp(double target) {
        return new SpinUp(target);
    }
    public Action maintainSpeed(double target) {
        return new MaintainSpeed(target);
    }
}
