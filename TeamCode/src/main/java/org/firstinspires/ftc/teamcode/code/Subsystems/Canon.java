package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Canon {
    public DcMotorEx motor;
    public DcMotor motor2;

    public int CLOSE_SPEED_FIRST = -1980;
    public int CLOSE_SPEED = -1900;
    public int targetVel = CLOSE_SPEED;

    public Canon(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "left_launcher");
        motor2 = (DcMotor)hardwareMap.get(DcMotorEx.class, "par");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double p) {
        motor.setPower(p);
        motor2.setPower(p);
    }

    private class SpinUp implements Action {
        private double targetSpeed;

        public SpinUp(double target) {
            targetSpeed = target;
        }

        public boolean run(TelemetryPacket t) {
            double speed = motor.getVelocity();
            double error = targetSpeed - speed;
            //motor.setPower(motor.getPower() + (error *ki));
            motor.setVelocity(CLOSE_SPEED);

            return Math.abs(error) > 40;
        }
    }

    private class CloneMotorPower implements Action {
        public boolean run(TelemetryPacket t) {
            motor2.setPower(motor.getPower());
            return false;
        }
    }

    public class SetPowerAction implements Action {
        private double power;
        public SetPowerAction(double pow) {
            power = pow;
        }
        public boolean run(TelemetryPacket t) {
            setPower(power);
            return false;
        }
    }

    public class SetVelAction implements Action {
        private double vel;
        public SetVelAction(double vel) {
            this.vel = vel;
        }
        public boolean run(TelemetryPacket t) {
            motor.setVelocity(vel);
            targetVel = (int)vel;
            return false;
        }
    }

    public class WaitUntilAtSpeed implements  Action {
        public boolean run(TelemetryPacket t) {
            boolean isAtSpeed = Math.abs(motor.getVelocity() - targetVel) <= 40;
            return !isAtSpeed;
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
    public Action cloneMotorPower() {
        return new CloneMotorPower();
    }
    public Action setPowerAction(double pow) {
        return new SetPowerAction(pow);
    }
    public Action waitUntilAtSpeed() {
        return new WaitUntilAtSpeed();
    }
    public Action setVelAction(double vel) {
        return new SetVelAction(vel);
    }
}
