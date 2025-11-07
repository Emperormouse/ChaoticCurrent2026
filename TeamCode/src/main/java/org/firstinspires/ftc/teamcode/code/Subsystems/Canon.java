package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Set;

public class Canon {
    public DcMotorEx motor;
    public int CLOSE_SPEED = -960;
    public int FAR_SPEED = -1100;
    public double closePower = 0;

    public Canon(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "left_launcher");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor.setVe

        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void setPower(double p) {
        motor.setPower(p);
    }

    private class SpinUp implements Action {
        private double targetSpeed;
        private final double ki = (1.0 / 25_000);

        public SpinUp(double target) {
            targetSpeed = target;
        }

        public boolean run(TelemetryPacket t) {
            double speed = motor.getVelocity();
            double error = targetSpeed - speed;
            //motor.setPower(motor.getPower() + (error *ki));
            motor.setVelocity(targetSpeed);

            return Math.abs(error) > 60;
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

    public class SetPowerInstant implements Action {
        private double power;
        public SetPowerInstant(double pow) {
            power = pow;
        }
        public boolean run(TelemetryPacket t) {
            motor.setPower(power);
            return false;
        }
    }

    public class SetVelInstant implements Action {
        private double vel;
        public SetVelInstant(double vel) {
            this.vel = vel;
        }
        public boolean run(TelemetryPacket t) {
            motor.setVelocity(vel);
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

    public Action setPowerInstant(double power) {
        return new SetPowerInstant(power);
    }

    public Action setVelInstant(double vel) {
        return new SetVelInstant(vel);
    }
}
