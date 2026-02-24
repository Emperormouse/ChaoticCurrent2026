package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.code.utility.Side;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Canon {
    public DcMotorEx motor;
    public DcMotor motor2;

    public int CLOSE_SPEED = 1200;
    public int CLOSE_SPEED_FIRST = CLOSE_SPEED;
    public int targetVel = CLOSE_SPEED;
    private Bot bot;
    public double CANON_OFFSET = 7.5;

    private double canonAngle = 55 * (Math.PI/180);

    public Canon(HardwareMap hardwareMap, Bot bot) {
        this.bot = bot;
        motor = hardwareMap.get(DcMotorEx.class, "left_launcher");
        motor2 = (DcMotor)hardwareMap.get(DcMotorEx.class, "par");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pid = new PIDFCoefficients(500.0, 0, 0 , 22.5);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
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

    public void setVelocityToCloseSpeed() {
        motor.setVelocity(CLOSE_SPEED);
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
        private double maxSpeed = 10_000;
        public SetVelByDistance(double maxSpeed) {
            this.maxSpeed = maxSpeed;
        }
        public boolean run(TelemetryPacket packet) {
            double x = bot.goalVec.x - bot.canonPose.position.x;
            double y = bot.goalVec.y - bot.canonPose.position.y;
            double z = 40 - 12; //goal height - canon height (constant)

            double d = Math.sqrt(x*x + y*y); //distance
            if (d < 42.0) //domain
                d = 42.0;

            double g = 386.2205; //g in inches/sec^2

            double t = Math.sqrt((2*(d*Math.tan(canonAngle)-z))/g);
            double translationalVelocity = (1/t)*Math.sqrt(d*d + Math.pow((z + 0.5*g*t*t), 2));

            double ticksPerSec = velocityToTicksPerSec2(translationalVelocity);

            motor.setVelocity(ticksPerSec);

            return true;
        }
    }
    public Action setVelByDistance(double maxSpeed) {
        return new SetVelByDistance(maxSpeed);
    }
    public Action setVelByDistance() {
        return new SetVelByDistance(10_000);
    }

    private double velocityToTicksPerSec(double v) {
        double a = 0.000276155;
        double b = -0.553527;
        double c = 429.32088;

        return (Math.sqrt(4*a*(v-c)+b*b) - b) / (2*a);
    }

    private double velocityToTicksPerSec2(double v) {
        return 6.98312*v + 17.21118;
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
