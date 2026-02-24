package org.firstinspires.ftc.teamcode.code.Subsystems;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.code.Subsystems.Canon;
import org.firstinspires.ftc.teamcode.code.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.code.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterFirstParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.KeepRunning;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;
import org.firstinspires.ftc.teamcode.code.utility.Op;
import org.firstinspires.ftc.teamcode.code.utility.Side;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class Bot {
    public Canon canon;
    public Gate gate;
    public Intake intake;
    public MecanumDrive drive;
    public Localizer localizer;

    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    public boolean useAprilTag = true;
    public Side side;

    public Pose2d launchPose;
    public Vector2d aprilVec;
    public Vector2d goalVec;
    public boolean isOpModeRunning = false;

    public Pose2d botPose = new Pose2d(0, 0, 0);
    public Pose2d canonPose = new Pose2d(0, 0, 0);

    Telemetry telemetry;
    HardwareMap hardwareMap;

    public Bot(HardwareMap hardwareMap, MecanumDrive drive, Side side, Telemetry telemetry) {
        canon = new Canon(hardwareMap, this);
        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap, this);
        this.drive = drive;
        this.localizer = drive.localizer;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.side = side;
        if (side == Side.BLUE) {
            launchPose = new Pose2d(-14.3, -9.8, Math.toRadians(51.5));
            aprilVec = new Vector2d(-58.3727f, -55.6425f);
            goalVec = new Vector2d(aprilVec.x-5, aprilVec.y-7);
        } else {
            launchPose = new Pose2d(-15.1, 14.8, Math.toRadians(-42.3));
            aprilVec = new Vector2d(-58.3727f, 55.6425f);
            goalVec = new Vector2d(aprilVec.x-5, aprilVec.y+7);
        }

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
    }

    public void initialize() {
        initAprilTag();
    }

    public Action shootClose(Op opmode) {
        if (opmode == Op.AUTO)
            return shootClose(opmode, 2.0, 0.4);
        else
            return shootClose(opmode, 100.0, 0.5);
    }

    public Action shootClose(Op opmode, double time1, double time2) {
        return new EndAfterFirstParallel(
            new SequentialAction(
                gate.open(),
                canon.waitUntilAtSpeed(),
                new Wait(0.3),
                new EndAfterFirstParallel(
                    new Wait(time1),
                    intake.setPower(-1.0)
                ),
                //canon.setPowerInstant(0),
                gate.close()
            ),
            new SequentialAction(
                canon.setVelAction(canon.CLOSE_SPEED)
            )
        );
    }

    /*public Action shootCloseNew() {
        return new SequentialAction(
            canon.spinUp(canon.CLOSE_SPEED + 10),
            gate.open(),
            new Wait(1.0),

            intake.intakeUntilBallShot(),
            new Wait(1.0),
            intake.intakeUntilBallShot(),
            new Wait(1.0),
            intake.intakeUntilBallShotLAST(),

            canon.setPowerInstant(0),
            gate.close()
        );
    }*/

    public void moveFieldCentric(double x, double y, double r, Op opmode) {
        moveFieldCentric(x, y, r, 1.0, opmode);
    }

    public void moveFieldCentric(double x, double y, double r, double speed, Op opmode) {
        double frPower = 0;
        double flPower = 0;
        double brPower = 0;
        double blPower = 0;

        double botRot = localizer.getPose().heading.toDouble();
        if (side == Side.BLUE && opmode == Op.TELE) {
            botRot += Math.toRadians(180);
        }

        frPower += r;
        brPower += r;
        flPower -= r;
        blPower -= r;

        //FORWARD-DIRECTION
        frPower += y * cos(botRot);
        brPower += y * cos(botRot);
        flPower += y * cos(botRot);
        blPower += y * cos(botRot);

        frPower -= y * sin(botRot);
        brPower += y * sin(botRot);
        flPower += y * sin(botRot);
        blPower -= y * sin(botRot);

        //SIDEWAYS-DIRECTION
        frPower += x * sin(botRot);
        brPower += x * sin(botRot);
        flPower += x * sin(botRot);
        blPower += x * sin(botRot);

        frPower += x * cos(botRot);
        brPower -= x * cos(botRot);
        flPower -= x * cos(botRot);
        blPower += x * cos(botRot);

        double denominator = max(1, max(max(max(abs(frPower), abs(brPower)), abs(flPower)), abs(blPower)));
        denominator /= speed;

        frontLeft.setPower(flPower / denominator);
        frontRight.setPower(frPower / denominator);
        backLeft.setPower(blPower / denominator);
        backRight.setPower(brPower / denominator);
    }
    public class MoveFieldCentricAction implements Action {
        double x, y, r, speed;
        public MoveFieldCentricAction(double x, double y, double r, double speed) {
            this.x = x;
            this.y = y;
            this.r = r;
            this.speed = speed;
        }

        public boolean run(TelemetryPacket t) {
            moveFieldCentric(x, y, r, speed, Op.AUTO);
            return true;
        }
    }
    public Action moveFieldCentricAction(double x,double y, double r, double speed) {
        return new MoveFieldCentricAction(x, y, r, speed);
    }

    public void moveRelative(double x, double y, double r, double speed) {
        double frPower = 0;
        double flPower = 0;
        double brPower = 0;
        double blPower = 0;

        frPower += r;
        brPower += r;
        flPower -= r;
        blPower -= r;

        //FORWARD-DIRECTION
        frPower += y;
        brPower += y;
        flPower += y;
        blPower += y;

        //SIDEWAYS-DIRECTION
        frPower += x;
        brPower -= x;
        flPower -= x;
        blPower += x;

        double denominator = max(1, max(max(max(abs(frPower), abs(brPower)), abs(flPower)), abs(blPower)));
        denominator /= speed;

        frontLeft.setPower(flPower / denominator);
        frontRight.setPower(frPower / denominator);
        backLeft.setPower(blPower / denominator);
        backRight.setPower(brPower / denominator);
    }
    public class MoveRelativeAction implements Action {
        double x, y, r, speed;
        public MoveRelativeAction(double x, double y, double r, double speed) {
            this.x = x;
            this.y = y;
            this.r = r;
            this.speed = speed;
        }

        public boolean run(TelemetryPacket t) {
            moveRelative(x, y, r, speed);
            return false;
        }
    }
    public Action moveRelativeAction(double x,double y, double r, double speed) {
        return new MoveRelativeAction(x, y, r, speed);
    }

    //drives to location
    public class MoveTo implements Action {
        private final double pRotational = 0.7;
        private final double pX = 0.05;
        private final double pY = 0.05;
        private final int turnMod;
        private Pose2d targetPose;
        private long lastTimeMoved = 0;
        private double speed;

        public MoveTo(Pose2d targetPose, double speed, int turnMod) {
            this.targetPose = targetPose;
            this.speed = speed;
            this.turnMod = turnMod;
        }

        public boolean run(TelemetryPacket t) {
            localizer.update();
            Pose2d currentPose = localizer.getPose();
            double diffX = targetPose.position.x - currentPose.position.x;
            double diffY = targetPose.position.y - currentPose.position.y;

            double targetR = targetPose.heading.toDouble();
            double currentR = currentPose.heading.toDouble();
            double diffR = targetR - currentR;
            telemetry.addData("diffR1: ", Math.toDegrees(diffR));
            if (turnMod == -1) {
                double diffR2 = targetR+(2*PI) - currentR;
                double diffR3 = targetR-(2*PI) - currentR;
                telemetry.addData("diffR2: ", Math.toDegrees(diffR2));
                telemetry.addData("diffR3: ", Math.toDegrees(diffR3));
                if (Math.abs(diffR2) < Math.abs(diffR))
                    diffR = diffR2;
                if (Math.abs(diffR3) < Math.abs(diffR))
                    diffR = diffR3;
            }

            double powX = diffX*pX;
            double powY = -diffY*pY;
            double powR = diffR*pRotational;

            moveFieldCentric(powX, powY, powR, speed, Op.AUTO);

            if (Math.abs(diffX) > 3.0 || Math.abs(diffY) > 3.0 || Math.abs(diffR) > Math.toRadians(3)) {
                lastTimeMoved = System.currentTimeMillis();
            }

            if ((Math.abs(diffX)>1.7 || Math.abs(diffY)>1.7 || Math.abs(diffR)>Math.toRadians(1.5)) &&
                System.currentTimeMillis() - lastTimeMoved < 2000)
            {
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public Action moveTo(Pose2d targetPose) {
        return new MoveTo(targetPose, 1.0, 1);
    }
    public Action moveTo(Pose2d targetPose, double speed) {
        return new MoveTo(targetPose, speed, 1);
    }
    public Action moveTo(Pose2d targetPose, double speed, int turnMod) {
        return new MoveTo(targetPose, speed, turnMod);
    }

    public class MoveToImprecise implements Action {
        private final double pRotational = 0.7;
        private final double pX = 0.05;
        private final double pY = 0.05;
        private Pose2d targetPose;
        private double speed;

        public MoveToImprecise(Pose2d targetPose, double speed) {
            this.targetPose = targetPose;
            this.speed = speed;
        }

        public boolean run(TelemetryPacket t) {
            localizer.update();
            Pose2d currentPose = localizer.getPose();
            double diffX = targetPose.position.x - currentPose.position.x;
            double diffY = targetPose.position.y - currentPose.position.y;
            double diffR = targetPose.heading.toDouble() - currentPose.heading.toDouble();

            double powX = diffX*pX;
            double powY = -diffY*pY;
            double powR = diffR*pRotational;

            moveFieldCentric(powX, powY, powR, speed, Op.AUTO);

            if (Math.abs(diffX)>2.0 || Math.abs(diffY)>2.0 || Math.abs(diffR)>Math.toRadians(2.5)) {
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public Action moveToImprecise(Pose2d targetPose) {
        return new MoveToImprecise(targetPose, 1.0);
    }
    public Action moveToImprecise(Pose2d targetPose, double speed) {
        return new MoveToImprecise(targetPose, speed);
    }


    public class MoveToContinuous implements Action {
        private final double pRotational = 0.7;
        private final double pX = 0.04;
        private final double pY = 0.04;
        private Pose2d targetPose;
        private double speed;

        public MoveToContinuous(Pose2d targetPose, double speed) {
            this.targetPose = targetPose;
            this.speed = speed;
        }

        public boolean run(TelemetryPacket t) {
            localizer.update();
            Pose2d currentPose = localizer.getPose();
            double diffX = targetPose.position.x - currentPose.position.x;
            double diffY = targetPose.position.y - currentPose.position.y;
            double diffR = targetPose.heading.toDouble() - currentPose.heading.toDouble();

            double powX = diffX*pX;
            double powY = -diffY*pY;
            double powR = diffR*pRotational;

            double minSpeed = 0.15;
            if (Math.abs(powX) < minSpeed && Math.abs(powY) < minSpeed && Math.abs(powR) < minSpeed) {
                double max;
                if (Math.abs(powX) > Math.abs(powY)) {
                    max = Math.abs(powX);
                }
                else {
                    max = Math.abs(powY);
                }
                double mod = minSpeed/max;
                powX *= mod;
                powY *= mod;
            }

            moveFieldCentric(powX, powY, powR, speed, Op.AUTO);

            if (Math.abs(diffX)>2.5 || Math.abs(diffY)>3 || Math.abs(diffR)>Math.toRadians(3.5)) {
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public Action moveToContinuous(Pose2d targetPose) {
        return new MoveToContinuous(targetPose, 1.0);
    }
    public Action moveToContinuous(Pose2d targetPose, double speed) {
        return new MoveToContinuous(targetPose, speed);
    }

    public class MoveToVeryImprecise implements Action {
        private final double pRotational = 0.8;
        private final double pX = 0.05;
        private final double pY = 0.05;
        private Pose2d targetPose;
        private double speed;

        public MoveToVeryImprecise(Pose2d targetPose, double speed) {
            this.targetPose = targetPose;
            this.speed = speed;
        }

        public boolean run(TelemetryPacket t) {
            localizer.update();
            Pose2d currentPose = localizer.getPose();
            double diffX = targetPose.position.x - currentPose.position.x;
            double diffY = targetPose.position.y - currentPose.position.y;
            double diffR = targetPose.heading.toDouble() - currentPose.heading.toDouble();

            moveFieldCentric(diffX*pX, -diffY*pY, diffR*pRotational, speed, Op.AUTO);

            if (Math.abs(diffX)>4.0 || Math.abs(diffY)>4.0 || Math.abs(diffR)>Math.toRadians(10)) {
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public Action moveToVeryImprecise(Pose2d targetPose) {
        return new MoveToVeryImprecise(targetPose, 1.0);
    }

    //infinitely settles
    public Action settle(Pose2d targetPose) {
        return new KeepRunning(new MoveTo(targetPose, 1.0, 1));
    }
    //settles for amount of time in seconds
    public Action settle(Pose2d targetPose, double time) {
        return new EndAfterFirstParallel(
            new Wait(time),
            settle(targetPose)
        );
    }

    //drives to location while tracking april tag
    public class MoveToTracked implements Action {
        private double pRotational = (1.0 / 70);
        private double pX = 0.04;
        private double pY = 0.04;
        private boolean reachedTargetVec = false;
        private Vector2d targetVec;
        private double speed;

        public MoveToTracked(Vector2d targetVec) {
            this(targetVec, 1.0);

        }
        public MoveToTracked(Vector2d targetVec, double speed) {
            this.targetVec = targetVec;
            this.speed = speed;
        }

        public boolean run(TelemetryPacket t) {
            localizer.update();
            Pose2d currentPose = localizer.getPose();

            //Tracking
            double dx = currentPose.position.x - goalVec.x;
            double dy = currentPose.position.y - goalVec.y;

            double targetAngle = Math.atan2(dx, -dy) - PI/2;
            if (targetAngle < -PI) {
                targetAngle += 2* PI;
            }
            if (side == Side.BLUE)
                targetAngle += Math.toRadians(2);


            double diffX = targetVec.x - currentPose.position.x;
            double diffY = targetVec.y - currentPose.position.y;
            double angleDiff = Math.toDegrees(targetAngle - currentPose.heading.toDouble());

            if (Math.abs(diffX) < 4.0 && Math.abs(diffY) < 4.0)
                reachedTargetVec = true;

            double powX = diffX*pX;
            double powY = -diffY*pY;
            double powR = angleDiff*pRotational;

            if (reachedTargetVec)
                moveFieldCentric(0, 0, powR, speed, Op.AUTO);
            else
                moveFieldCentric(powX, powY, powR, speed, Op.AUTO);

            if (!reachedTargetVec || Math.abs(angleDiff)>5.0) {
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public Action moveToTracked(Vector2d targetVec) {
        return new MoveToTracked(targetVec);
    }
    public Action moveToTracked(Vector2d targetVec, double speed) {
        return new MoveToTracked(targetVec, speed);
    }

    //drives to location while tracking april tag
    public class AimAtGoal implements Action {
        private double pRotational = (1.0 / 60);
        private double speed;

        public AimAtGoal() {
            this(1.0);
        }
        public AimAtGoal(double speed) {
            this.speed = speed;
        }

        public boolean run(TelemetryPacket t) {
            localizer.update();
            Pose2d currentPose = localizer.getPose();

            //Tracking
            double dx = currentPose.position.x - goalVec.x;
            double dy = currentPose.position.y - goalVec.y;

            double targetAngle = Math.atan2(dx, -dy) - PI/2;
            if (targetAngle < -PI) {
                targetAngle += 2* PI;
            }
            if (side == Side.BLUE)
                targetAngle += Math.toRadians(2);

            double angleDiff = Math.toDegrees(targetAngle - currentPose.heading.toDouble());
            double powR = angleDiff*pRotational;

            moveRelative(0, 0, powR, speed);

            if (Math.abs(angleDiff)>5.0) {
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public Action aimAtGoal() {
        return new AimAtGoal();
    }
    public Action aimAtGoal(double speed) {
        return new AimAtGoal(speed);
    }

    public double targetDistance = 52;

    public class MoveToLaunchArc implements Action {
        double ky = (1.0 / 60);
        double kr = (1.0 / 450);
        double kr2 = (1.0 / 60);
        boolean subArc;

        public MoveToLaunchArc(boolean subArc) {
            this.subArc = subArc;
        }

        public boolean run(TelemetryPacket telemetryPacket) {
            double x = 0;
            double y = 0;
            double r = 0;

            double distanceDiff = 0;
            double offset = 0;
            double angleDiff = 0;

            Vector2d aprilVec;
            Vector2d goalVec;
            if (side == Side.RED) {
                aprilVec = new Vector2d(-58.3727f, 55.6425f);
                goalVec = new Vector2d(aprilVec.x-7, aprilVec.y+7);
            } else {
                aprilVec = new Vector2d(-58.3727f, -55.6425f);
                goalVec = new Vector2d(aprilVec.x-7, aprilVec.y-7);
            }

            Pose2d botPose = localizer.getPose();
            double dx = botPose.position.x - aprilVec.x;
            double dy = botPose.position.y - aprilVec.y;
            double dxGoal = botPose.position.x - goalVec.x;
            double dyGoal = botPose.position.y - goalVec.y;

            AprilTagDetection detection = getLatestAprilTagDetection();
            boolean found = (detection != null);

            double targetAngle = Math.atan2(dxGoal, -dyGoal) - PI/2;
            if (targetAngle < -PI) {
                targetAngle += 2* PI;
            }
            angleDiff = targetAngle - botPose.heading.toDouble();
            r = Math.toDegrees(angleDiff) * kr2;

            double distance = Math.sqrt(dx*dx + dy*dy);
            distanceDiff = targetDistance - distance;
            y = distanceDiff * ky;

            telemetry.addData("targetAngle: ", Math.toDegrees(targetAngle));
            telemetry.addData("actAngle: ", Math.toDegrees(botPose.heading.toDouble()));

            telemetry.addData("dx: ", dx);
            telemetry.addData("dy: ", dy);


            double angle2 = Math.toDegrees(Math.atan2(Math.abs(dy), Math.abs(dx)));
            double targetAngle2 = 50;
            double targetAngle2Diff = targetAngle2 - angle2;

            x = targetAngle2Diff * (1.0 / 35);
            if (side == Side.RED)
                x *= -1;


            telemetry.addData("distanceDiff: ", distanceDiff);
            telemetry.addData("offset: ", offset);
            telemetry.addData("angle2: ", angle2);

            telemetry.addData("x: ", x);
            telemetry.addData("y: ", y);
            telemetry.addData("r: ", r);
            telemetry.update();

            if (subArc) {
                if (found || Math.abs(Math.toDegrees(angleDiff)) < 45) {
                    moveRelative(x, y, r, 0.8);
                } else {
                    moveRelative(0, 0, r, 0.8);
                }

                if (!found || Math.abs(distanceDiff) > 6 || Math.abs(offset) > 20 || angle2 < Math.toDegrees(40) || angle2 > Math.toDegrees(60)) {
                    return true;
                } else {
                    stop();
                    return false;
                }
            } else {
                if (found || Math.abs(Math.toDegrees(angleDiff)) < 45) {
                    moveRelative(0, y, r, 0.8);
                } else {
                    moveRelative(0, 0, r, 0.8);
                }

                if (!found || Math.abs(distanceDiff) > 6 || Math.abs(offset) > 20) {
                    return true;
                } else {
                    stop();
                    return false;
                }
            }
        }
    }
    public Action moveToLaunchSubArc() {
        return new MoveToLaunchArc(true);
    }
    public Action moveToLaunchArc() {
        return new MoveToLaunchArc(false);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public class Stop implements Action {
        public boolean run(TelemetryPacket t) {
            stop();
            return false;
        }
    }
    public Action stopAction() {
        return new Stop();
    }

    // ===== APRIL TAG =====

    public void updatePose() {
        Pose2d aprilPose = getPoseFromAprilTag();
        Pose2d p = localizer.getPose();
        if (aprilPose != null) {
            localizer.setPose(aprilPose);
            p = aprilPose;
        }
        localizer.update();
        drive.updatePoseEstimate();

        botPose = new Pose2d(p.position.x, p.position.y, p.heading.toDouble());
        double x = botPose.position.x;
        double y = botPose.position.y;
        double r = botPose.heading.toDouble();
        canonPose = new Pose2d(x+cos(r)*canon.CANON_OFFSET, y+sin(r)*canon.CANON_OFFSET, r);
    }

    private class SetUseAprilTag implements Action {
        boolean isEnabled;
        public SetUseAprilTag(boolean b) {
            isEnabled = b;
        }
        public boolean run(TelemetryPacket t) {
            useAprilTag = isEnabled;
            return false;
        }
    }
    public Action enableAprilTag() {
        return new SetUseAprilTag(true);
    }
    public Action disableAprilTag() {
        return new SetUseAprilTag(false);
    }

    public void updatePoseUsingAprilTag() {
        Pose2d aprilPose = getPoseFromAprilTag();
        if (aprilPose != null) {
            lastTimeTagSeen = System.currentTimeMillis();
            localizer.setPose(aprilPose);
        }
        localizer.update();
        drive.updatePoseEstimate();
    }

    public class UpdatePoseUsingAprilTagAction implements Action {
        public boolean run(TelemetryPacket t) {
            updatePoseUsingAprilTag();
            return true;
        }
    }
    public Action updatePoseUsingAprilTagAction() {
        return new UpdatePoseUsingAprilTagAction();
    }

    public Pose2d getPoseFromAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getFreshDetections();
        if (!useAprilTag)
            return null;
        if (currentDetections == null)
            return null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 20 || detection.id == 24) {
                    return new Pose2d(
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) - Math.toRadians(90)
                    );
                }
            }
        }
        return null;
    }

    private long lastTimeTagSeen = 0;
    public class WaitUntilSeeTag implements Action {
        double time;
        public WaitUntilSeeTag(double time) {
            this.time = time;
        }
        public WaitUntilSeeTag() {
            this.time = 0;
        }
        public boolean run(TelemetryPacket t) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == 20 || detection.id == 24) {
                        lastTimeTagSeen = System.currentTimeMillis();
                        return false;
                    }
                }
            }
            if (lastTimeTagSeen + time*1000 > System.currentTimeMillis()) {
                return false;
            }
            return true;
        }
    }
    public Action waitUntilSeeTag() {
        return new WaitUntilSeeTag();
    }
    public Action waitUntilSeeTag(double time) {
        return new WaitUntilSeeTag(time);
    }

    public AprilTagDetection getLatestAprilTagDetection() {
        int targetId = (side == Side.BLUE) ? 20 : 24;

        ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.id == targetId) {
                return detection;
            }
        }

        return null;
    }

    public void aprilTagTelementary() {
        telemetry.addData("LocalizerX: ", drive.localizer.getPose().position.x);
        telemetry.addData("LocalizerY: ", drive.localizer.getPose().position.y);
        telemetry.addData("LocalizerR: ", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addData("Offset of April Tag: ", detection.center);
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                    detection.robotPose.getPosition().x,
                    detection.robotPose.getPosition().y,
                    detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                    detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                    detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                    detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)-90));
            }
        }
    }
    public class TelementaryAction implements Action {
        public boolean run(TelemetryPacket t) {
            aprilTagTelementary();
            telemetry.update();

            return true;
        }
    }
    public Action telementaryAction() {
        return new TelementaryAction();
    }

    public static AprilTagLibrary getDecodeTagLibrary(){
        return new AprilTagLibrary.Builder()
            .addTag(20, "BlueTarget",
                6.5, new VectorF(-58.3727f, -55.6425f, 29.5f), DistanceUnit.INCH,
                new Quaternion(0.2182149f, -0.2182149f, -0.6725937f, 0.6725937f, 0))
            .addTag(21, "Obelisk_GPP",
                6.5, DistanceUnit.INCH)
            .addTag(22, "Obelisk_PGP",
                6.5, DistanceUnit.INCH)
            .addTag(23, "Obelisk_PPG",
                6.5, DistanceUnit.INCH)
            .addTag(24, "RedTarget",
                6.5, new VectorF(-58.3727f, 55.6425f, 29.5f), DistanceUnit.INCH,
                new Quaternion(0.6725937f, -0.6725937f, -0.2182149f, 0.2182149f, 0))
            .build();
    }

    public Position cameraPosition = new Position(DistanceUnit.INCH, 0, 6.5, 6.5, 0);
    public YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
    public void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

            // The following default settings are available to un-comment and edit as needed.
            //.setDrawAxes(false)
            //.setDrawCubeProjection(false)
            //.setDrawTagOutline(true)
            //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(getDecodeTagLibrary())
            //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .setCameraPose(cameraPosition, cameraOrientation)

            // == CAMERA CALIBRATION ==
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera.
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
            // ... these parameters are fx, fy, cx, cy.

            .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

}
