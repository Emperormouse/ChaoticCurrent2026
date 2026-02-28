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

import org.firstinspires.ftc.teamcode.code.utility.QuarticSolver;
import org.firstinspires.ftc.teamcode.code.utility.Side;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Canon {
    public DcMotorEx motor;
    public DcMotor motor2;

    public int CLOSE_SPEED = 1200;
    public int CLOSE_SPEED_FIRST = CLOSE_SPEED;
    public int targetVel = CLOSE_SPEED;
    public double targetAngle = -1.0;
    private Bot bot;
    public double CANON_OFFSET = -7.5;

    private double canonAngle = 55 * (Math.PI/180);

    public Canon(HardwareMap hardwareMap, Bot bot) {
        this.bot = bot;
        motor = hardwareMap.get(DcMotorEx.class, "left_launcher");
        motor2 = (DcMotor)hardwareMap.get(DcMotorEx.class, "par");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients pid = new PIDFCoefficients(700.0, 0, 0 , 22.5);
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
            double z = 43 - 12; //goal height - canon height (constant)

            double d = Math.sqrt(x*x + y*y); //distance
            if (d < 42.0) //domain
                d = 42.0;

            double g = 386.2205; //g in inches/sec^2

            double t = Math.sqrt((2*(d*Math.tan(canonAngle)-z))/g);
            double translationalVelocity = (1/t)*Math.sqrt(d*d + Math.pow((z + 0.5*g*t*t), 2));

            double ticksPerSec = velocityToTicksPerSec2(translationalVelocity);

            //adjustment for far zone
            if (d > 90.0)
                ticksPerSec += 40;

            targetVel = (int)ticksPerSec;
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

    public class SetVelByDistanceWithRSpeed implements Action {
        private double maxSpeed = 10_000;
        public SetVelByDistanceWithRSpeed(double maxSpeed) {
            this.maxSpeed = maxSpeed;
        }
        public boolean run(TelemetryPacket packet) {
            double x = bot.goalVec.x - bot.canonPose.position.x;
            double y = bot.goalVec.y - bot.canonPose.position.y;
            double z = 43 - 12; //goal height - canon height (constant)

            double Rx = bot.velX;
            double Ry = bot.velY;

            double g = 386.2205; //g in inches/sec^2

            double tanA = Math.tan(canonAngle);

            double a = 0.25 * g * g;
            double b = g * z - tanA * tanA * (Rx*Rx + Ry*Ry);
            double c = 2 * tanA * tanA * (x*Rx + y*Ry);
            double d = z*z - tanA * tanA * (x*x + y*y);

            Double t = QuarticSolver.lowestPositiveRealSolution(a, b, c, d);
            if (t == null) {
                targetVel = 10_000;
                return false;
            }

            double Vx = (x/t)-Rx;
            double Vy = (y/t)-Ry;
            double Vz = (z + 0.5*g*t*t)/t;

            targetAngle = Math.atan2(Vy, Vx);

            double translationalVelocity = Math.sqrt(Vx*Vx + Vy*Vy + Vz*Vz);

            double ticksPerSec = velocityToTicksPerSec2(translationalVelocity);

            targetVel = (int)ticksPerSec;
            motor.setVelocity(ticksPerSec);

            return true;
        }
    }
    public Action setVelByDistanceWithRSpeed(double maxSpeed) {
        return new SetVelByDistanceWithRSpeed(maxSpeed);
    }
    public Action setVelByDistanceWithRSpeed() {
        return new SetVelByDistanceWithRSpeed(10_000);
    }

    public boolean isAtSpeed() {
        return Math.abs(targetVel - motor.getVelocity()) <= 60;
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
