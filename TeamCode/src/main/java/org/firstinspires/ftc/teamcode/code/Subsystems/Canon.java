package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Canon {
    public DcMotorEx motor;
    public final int FAR_SPEED = -1200;

    public Canon(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "left_launcher");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorR = motor;
    }
    
    public void setPower(double p) {
        motor.setPower(-p);
    }

    private class SpinUp implements Action {
        private double targetSpeed;
        private final double ki = (1.0 / 20_000);

        public SpinUp(double target) {
            targetSpeed = target;
        }

        public boolean run(TelemetryPacket t) {
            double speed = motor.getVelocity();
            double error = targetSpeed - speed;
            motor.setPower(motor.getPower() + (error *ki));

            return Math.abs(error) > 20;
        }
    }

    private class MaintainSpeed extends SpinUp {
        public MaintainSpeed(double target) {
            super(target);
        }
        public boolean run(TelemetryPacket t) {
            super.run(t);
            return true;
        }
    }

    public class StopInstant implements Action {
        public boolean run(TelemetryPacket t) {
            motor.setPower(0);
            return false;
        }
    }



    private class ActionName implements Action { //Action Structure
        private int a;
        public ActionName() {

        }
        public boolean run(TelemetryPacket t) {

            return true;
        }
    }

    public Action spinUp(double target) {
        return new SpinUp(target);
    }
    public Action maintainSpeed(double target) {
        return new MaintainSpeed(target);
    }
    public Action stopInstant() {
        return new StopInstant();
    }
}
