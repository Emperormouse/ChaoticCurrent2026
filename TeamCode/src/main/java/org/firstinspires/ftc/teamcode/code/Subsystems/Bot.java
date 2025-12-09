package org.firstinspires.ftc.teamcode.code.Subsystems;

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
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterFirstParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.KeepRunning;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;
import org.firstinspires.ftc.teamcode.code.utility.Op;
import org.firstinspires.ftc.teamcode.code.utility.Side;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Bot {
    public Canon canon;
    public Gate gate;
    public Intake intake;
    public DistanceSensor distanceSensor;
    MecanumDrive drive;
    public Localizer localizer;

    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    public YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    public boolean useAprilTag = true;
    public Side side;

    public Pose2d launchPose;

    Telemetry telemetry;
    HardwareMap hardwareMap;

    public Bot(HardwareMap hardwareMap, MecanumDrive drive, Side side, Telemetry telemetry) {
        canon = new Canon(hardwareMap);
        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap, this);
        this.drive = drive;
        this.localizer = drive.localizer;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.side = side;
        if (side == Side.BLUE)
            launchPose = new Pose2d(-14.3, -9.8, Math.toRadians(51.5));
        else
            launchPose = new Pose2d(-15.1, 14.8, Math.toRadians(-42.3));

        frontLeft = (DcMotor) hardwareMap.get(DcMotor.class, "front_left");
        backLeft = (DcMotor)hardwareMap.get(DcMotor.class, "back_left");
        frontRight = (DcMotor)hardwareMap.get(DcMotor.class, "front_right");
        backRight = (DcMotor)hardwareMap.get(DcMotor.class, "back_right");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
    }

    public void initialize() {
        initAprilTag();

        /*Actions.runBlocking(new SequentialAction(
            canon.spinUp(canon.CLOSE_SPEED),
            new EndAfterFirstParallel(
                new Wait(7.0),
                canon.maintainSpeed(canon.CLOSE_SPEED)
            )
        ));
        canon.closePower = canon.motor.getPower();
        canon.setPower(0);*/
    }

    public Action shootClose(Op opmode) {
        double time = (opmode == Op.AUTO) ? 4.2 : 10;
        return new EndAfterFirstParallel(
            new SequentialAction(
                gate.open(),
                new Wait(0.8),
                new EndAfterFirstParallel(
                    new Wait(time),
                    intake.intakeWhenAtSpeed()
                ),
                //canon.setPowerInstant(0),
                gate.close()
            ),
            new SequentialAction(
                canon.setVelAction(canon.CLOSE_SPEED_FIRST),
                new Wait(2),
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

        double botRot = localizer.getPose().heading.toDouble() + Math.toRadians(90);
        if (side == Side.RED && opmode == Op.TELE) {
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
        private final double pRotational = 1.0;
        private final double pX = 0.065;
        private final double pY = 0.065;
        private Pose2d targetPose;
        private long lastTimeMoved = 0;
        private double speed;

        public MoveTo(Pose2d targetPose, double speed) {
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

            if (Math.abs(diffX) > 3.0 || Math.abs(diffY) > 3.0 || Math.abs(diffR) > Math.toRadians(2.8)) {
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
        return new MoveTo(targetPose, 1.0);
    }
    public Action moveTo(Pose2d targetPose, double speed) {
        return new MoveTo(targetPose, speed);
    }

    public class MoveToImprecise implements Action {
        private final double pRotational = 1.0;
        private final double pX = 0.08;
        private final double pY = 0.08;
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

            moveFieldCentric(diffX*pX, -diffY*pY, diffR*pRotational, speed, Op.AUTO);

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

    public class MoveToVeryImprecise implements Action {
        private final double pRotational = 1.0;
        private final double pX = 0.08;
        private final double pY = 0.08;
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

            if (Math.abs(diffX)>3.0 || Math.abs(diffY)>3.0 || Math.abs(diffR)>Math.toRadians(5)) {
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
        return new KeepRunning(new MoveTo(targetPose, 1.0));
    }
    //settles for amount of time in seconds
    public Action settle(Pose2d targetPose, double time) {
        return new EndAfterFirstParallel(
            new Wait(time),
            settle(targetPose)
        );
    }

    public double targetDistance = 52.0;

    public class MoveToLaunchArc implements Action {
        double ky = (1.0 / 60);
        double kr = (1.0 / 450);
        double kr2 = (1.0 / 80);

        public boolean run(TelemetryPacket telemetryPacket) {
            double x = 0;
            double y = 0;
            double r = 0;

            double distanceDiff = 0;
            double offset = 0;
            double angleDiff = 0;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            int targetId;
            Vector2d goalVec;
            if (side == Side.RED) {
                targetId = 24;
                goalVec = new Vector2d(-58.3727f, 55.6425f);
            } else {
                targetId = 20;
                goalVec = new Vector2d(-58.3727f, -55.6425f);
            }

            Pose2d botPose = localizer.getPose();
            double dx = botPose.position.x - goalVec.x;
            double dy = botPose.position.y - goalVec.y;

            boolean found = false;
            for (AprilTagDetection aprilTag : currentDetections) {
                if (aprilTag.metadata != null) {
                    if (aprilTag.id == targetId) {
                        found = true;
                        offset = -aprilTag.center.x + 300;
                        r = offset * kr;
                        if (side == Side.RED)
                            r *= -1;

                        double distance = aprilTag.ftcPose.y;
                        distanceDiff = targetDistance - distance;
                        y = distanceDiff * ky;
                        if (side == Side.RED)
                            y *= -1;

                        telemetry.addData("Distance: ", distance);
                    }
                }
            }

            if (!found) {
                double targetAngle = Math.atan2(dx, -dy) - Math.PI/2;
                if (targetAngle < -Math.PI) {
                    targetAngle += 2*Math.PI;
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
            }

            double angle2 = Math.toDegrees(Math.atan2(Math.abs(dy), Math.abs(dx)));
            double targetAngle2 = 50;
            double targetAngle2Diff = targetAngle2 - angle2;

            x = targetAngle2Diff * (1.0 / 35);
            if (side == Side.RED)
                x *= -1;

            if (found || Math.abs(Math.toDegrees(angleDiff)) < 45) {
                //moveRelative(x, y, r, 1.0);
            } else {
                //moveRelative(0, 0, r, 1.0);
            }
            telemetry.addData("distanceDiff: ", distanceDiff);
            telemetry.addData("offset: ", offset);
            telemetry.addData("angle2: ", angle2);

            telemetry.addData("x: ", x);
            telemetry.addData("y: ", y);
            telemetry.addData("r: ", r);
            telemetry.update();

            if (!found || Math.abs(distanceDiff) > 6 || Math.abs(offset) > 20 || angle2 < Math.toDegrees(40) || angle2 > Math.toDegrees(60)) {
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public Action moveToLaunchArc() {
        return new MoveToLaunchArc();
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

    public void updatePoseUsingAprilTag() {
        if (useAprilTag) {
            Pose2d aprilPose = getPoseFromAprilTag();
            if (aprilPose != null) {
                localizer.setPose(aprilPose);
            }
        }
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
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
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

    public void aprilTagTelementary() {
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
