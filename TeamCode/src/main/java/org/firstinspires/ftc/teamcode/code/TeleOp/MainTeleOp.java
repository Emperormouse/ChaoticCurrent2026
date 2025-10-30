package org.firstinspires.ftc.teamcode.code.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.code.Subsystems.Bot;
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterFirstParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//This is a teleOp that I'm working on which will be able to run RoadRunner paths in teleOp.
//The best way that I figured out to do this is to make every part of the teleOp an Action, which
//is what roadrunner paths are, and then switch between those actions.
//This will most likely be used if we want to make a TeleOp which is mostly automated
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private IMU imu;
    private Bot bot;
    private Pose2d currentPose;
    private boolean isOuttaking = false;
    private double pow = 0.0;

    //This is the roadrunner mecanum drive
    MecanumDrive drive;

    //MANUAL CONTROLS BESIDES MOVEMENT IN HERE
    private class ManualControls implements Action {
        public boolean run(@NonNull TelemetryPacket t) {
            if (gamepad1.a) {
                telemetry.addLine("A pressed");
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                bot.intake.intake();
            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                bot.intake.reverse();
            } else {
                bot.intake.stop();
            }

            if (gamepad1.a || gamepad2.a) {
                bot.gate.openManual();
            } else if (gamepad1.b || gamepad2.b) {
                bot.gate.closeManual();
            } else {
                bot.gate.holdManual();
            }

            return true;
        }
    }

    //This is code for field-centric movement, which means that the robot will move the same
    //way regardless of the direction that its facing. It's meant to be called many times in a loop
    //It's structured as an Action however since this teleOp is based on roadrunner Actions
    private class FieldCentricMovement implements Action {
        public boolean run(@NonNull TelemetryPacket t) {
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            if (isOuttaking)
                rx /= 3;

            double speed = (gamepad1.left_bumper) ? 0.6 : 1.0;

            bot.moveFieldCentric(x, y, rx, speed);
            return true;
        }
    }
    //END OF MANUAL CONTROLS

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(60.1, -12.7, 0);
        drive = new MecanumDrive(hardwareMap, startPose); //The start position,
        // which is on the left tile of the far launch zone, with the intake up against the wall.

        bot = new Bot(hardwareMap, drive.localizer);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        ));
        imu.resetYaw();

        initAprilTag();
        TelemetryPacket t = new TelemetryPacket();

        Actions.runBlocking(new SequentialAction(
            bot.canon.spinUp(bot.canon.CLOSE_SPEED),
            new EndAfterFirstParallel(
                new Wait(7.0),
                bot.canon.maintainSpeed(bot.canon.CLOSE_SPEED)
            )
        ));
        bot.canon.closePower = bot.canon.motor.getPower();
        pow = bot.canon.motor.getPower();
        bot.canon.setPower(0);

        Action defaultAction = new ParallelAction(
            new FieldCentricMovement(),
            new ManualControls()
        );
        Action currentAction = defaultAction;

        waitForStart();
        while (opModeIsActive()) {
            telemetry.update();
            telemetry.addData("Localer: ", drive.localizer.getPose());
            aprilTagTelementary();

            Pose2d aprilPose = getPoseFromAprilTag();
            if (aprilPose != null) {
                drive.localizer.setPose(aprilPose);
            }
            drive.updatePoseEstimate();
            currentPose = drive.localizer.getPose();

            /*if (Math.abs(bot.canon.motor.getVelocity() - bot.canon.CLOSE_SPEED) <= 20) {
                gamepad1.setLedColor(0, 255, 0, 1);
            } else {
                gamepad1.setLedColor(255, 0, 0, 1);
            }*/

            if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
                if (isOuttaking || Math.abs(bot.canon.motor.getVelocity())>800) {
                    isOuttaking = false;
                    bot.canon.setPower(0);
                    bot.gate.close();
                } else {
                    isOuttaking = true;
                    bot.canon.setPower(bot.canon.closePower);
                }
            }
            if (gamepad1.xWasPressed()) {
                currentAction = new SequentialAction(
                    new ParallelAction(
                        pathToLaunchPosClose(currentPose),
                        new SequentialAction(
                            bot.gate.open(),
                            new Wait(0.3),
                            bot.gate.close(),
                            new Wait(0.3),
                            bot.canon.spinUp(bot.canon.CLOSE_SPEED)
                        )
                    ),
                    bot.shootClose()
                );
            }
            if (gamepad1.yWasPressed()) {
                currentAction = pathToLaunchPos(currentPose);
            }
            if (gamepad1.dpadRightWasPressed()) {
                currentAction = new ParallelAction(
                    currentAction,
                    bot.shootClose(),
                    new FieldCentricMovement()
                );
            }
            if (gamepad1.dpadDownWasPressed()) {
                currentAction = pathToPos(currentPose, startPose);
            }
            if (gamepad1.bWasPressed()) {
                bot.canon.setPower(0);
                currentAction = defaultAction;
            }

            if (!currentAction.run(t)) {
                currentAction = defaultAction;
            }

            telemetry.addData("Target Speed: ", bot.canon.FAR_SPEED);
            telemetry.addData("Canon power: ", bot.canon.motor.getPower());
            telemetry.addData("Canon speed: ", bot.canon.motor.getVelocity());
            telemetry.addData("Pos: ", currentPose);
            telemetry.addData("Rotation: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("TargetPow", pow);
        }
    }

    private Pose2d getPoseFromAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 20 || detection.id == 24) {
                    return new Pose2d(
                        detection.robotPose.getPosition().x + 5.0,
                        detection.robotPose.getPosition().y - 8.1,
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
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                    detection.robotPose.getPosition().x+5.0,
                    detection.robotPose.getPosition().y-8.1,
                    detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                    detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                    detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                    detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)-90));
            }
        }
    }

    //A roadrunner path
    public Action pathToLaunchPos(Pose2d startPose) {
        //A path which moves to the point
        return drive.actionBuilder(startPose)
            .strafeToSplineHeading(new Vector2d(44, -9), Math.toRadians(24.5))
            .build();
    }

    public Action pathToLaunchPosClose(Pose2d startPose) {
        //A path which moves to the point
        return drive.actionBuilder(startPose)
            .strafeToSplineHeading(new Vector2d(-6, -9), Math.toRadians(43.7))
            .build();
    }

    public Action pathToPos(Pose2d startPose, Pose2d target) {
        return drive.actionBuilder(startPose)
            .strafeToSplineHeading(target.position, target.heading.toDouble())
            .build();
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

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private void initAprilTag() {

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

