package org.firstinspires.ftc.teamcode.examples.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    private final DcMotor motor;

    public Slides (HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "slides");
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public class MoveSlides implements Action {
        final double kp = 0.002;
        int targetTicks;

        public MoveSlides(int targetTicks) {
            this.targetTicks = targetTicks;
        }

        public boolean run(@NonNull TelemetryPacket t) {
            int error = targetTicks - motor.getCurrentPosition();
            motor.setPower(Math.min(1, error*kp));

            return Math.abs(error) > 10;
        }
    }
    public Action moveSlides(int targetTicks) {
        return new MoveSlides(targetTicks);
    }

}
