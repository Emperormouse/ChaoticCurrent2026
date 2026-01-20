package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.code.utility.Side;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Canon {
    public DcMotorEx motor;
    public DcMotor motor2;

    public int CLOSE_SPEED = -1880;
    public int CLOSE_SPEED_FIRST = CLOSE_SPEED;
    public int targetVel = CLOSE_SPEED;
    private Bot bot;

    public Canon(HardwareMap hardwareMap, Bot bot) {
        this.bot = bot;
        motor = hardwareMap.get(DcMotorEx.class, "left_launcher");
        motor2 = (DcMotor)hardwareMap.get(DcMotorEx.class, "par");

        //motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double p) {
        motor.setPower(p);
        motor2.setPower(p);
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

    public class SetVelByDistance implements Action {
        public boolean run(TelemetryPacket t) {
            AprilTagDetection detection = bot.getLatestAprilTagDetection();
            double distance;

            if (detection != null) {
                distance = detection.ftcPose.y;
            } else {
                Vector2d goalVec;
                if (bot.side == Side.RED) {
                    goalVec = new Vector2d(-58.3727f, 55.6425f);
                } else {
                    goalVec = new Vector2d(-58.3727f, -55.6425f);
                }

                Pose2d botPose = bot.localizer.getPose();
                double dx = botPose.position.x - goalVec.x;
                double dy = botPose.position.y - goalVec.y;

                distance = Math.sqrt(dx*dx + dy*dy);
            }

            double m = -5.276;
            double c = -1604;
            double velocity = (m*distance) + c;

            motor.setVelocity(velocity);

            return true;
        }
    }
    public Action setVelByDistance() {
        return new SetVelByDistance();
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

    public Action cloneMotorPower() {
        return new CloneMotorPower();
    }
    public Action setPowerAction(double pow) {
        return new SetPowerAction(pow);
    }
    public Action setVelAction(double vel) {
        return new SetVelAction(vel);
    }
    public Action waitUntilAtSpeed() {
        return new WaitUntilAtSpeed();
    }
}
