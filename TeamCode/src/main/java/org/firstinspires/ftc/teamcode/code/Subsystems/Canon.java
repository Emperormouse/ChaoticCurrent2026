package org.firstinspires.ftc.teamcode.code.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Set;

public class Canon {
    public DcMotorEx motorL;
    public DcMotorEx motorR;

    public Canon(HardwareMap hardwareMap) {
        motorL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        motorR = hardwareMap.get(DcMotorEx.class, "outtakeR");
        motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorR = motorL;
    }
    
    public void setPower(double p) {
        motorL.setPower(-p);
        motorR.setPower(p);
    }

    private class SpinUp implements Action {
        private double targetSpeed;
        private final double k = (1.0 / 50_000);

        public SpinUp(double target) {
            targetSpeed = target;
        }

        public boolean run(TelemetryPacket t) {
            double speedR = motorR.getVelocity();
            double errorR = targetSpeed - speedR;
            motorL.setPower(motorL.getPower() + (errorR*k));

            double speedL = motorL.getVelocity();
            double errorL = targetSpeed - speedL;
            motorL.setPower(motorL.getPower() + (errorL*k));

            return Math.abs(errorR) > 20;
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
}
