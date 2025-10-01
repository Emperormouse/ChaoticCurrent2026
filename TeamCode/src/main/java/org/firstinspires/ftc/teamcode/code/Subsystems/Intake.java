package org.firstinspires.ftc.teamcode.code.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;
public class Intake {
    private DcMotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void startIntake() {
        motor.setPower(1.0);
    }

    public void reverse() {
        motor.setPower(-1.0);
    }

    public void stop() {
        motor.setPower(0.0);
    }

    public class setSpeed implements Action {
        double usableSpeed;

        public setSpeed(double targetSpeed) {
            usableSpeed = targetSpeed;
        }

        public boolean run(@NonNull TelemetryPacket t) {
            double error1 = usableSpeed - motor.getVelocity();
            motor.setPower(motor.getPower() + (error1 * 0.00002));

            return (Math.abs(error1) > 15);
        }

        public class MaintainSpeed implements Action {
            double usableSpeed;

            public MaintainSpeed(double targetSpeed) {
                usableSpeed = targetSpeed;
            }

            public boolean run(@NonNull TelemetryPacket t) {
                double error1 = usableSpeed - motor.getVelocity();
                motor.setPower(motor.getPower() + (error1 * 0.00002));

                return true;
            }

            public Action setSpeed(double targetSpeed) {
                return new setSpeed(targetSpeed);
            }

            public Action maintainSpeed(double targetSpeed) {
                return new MaintainSpeed(targetSpeed);
            }


        }
    }

}
