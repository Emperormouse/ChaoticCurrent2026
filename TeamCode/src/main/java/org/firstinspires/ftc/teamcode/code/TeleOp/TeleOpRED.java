package org.firstinspires.ftc.teamcode.code.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.code.Subsystems.Bot;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;
import org.firstinspires.ftc.teamcode.code.utility.Op;
import org.firstinspires.ftc.teamcode.code.utility.Side;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//This is a teleOp that I'm working on which will be able to run RoadRunner paths in teleOp.
//The best way that I figured out to do this is to make every part of the teleOp an Action, which
//is what roadrunner paths are, and then switch between those actions.
//This will most likely be used if we want to make a TeleOp which is mostly automated
@TeleOp
public class TeleOpRED extends LinearOpMode {
    private Bot bot;
    private Pose2d currentPose;
    private boolean isOuttaking = false;
    private boolean useAprilTag = true;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    Side side = Side.RED;

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

            if (gamepad2.a) {
                bot.gate.openManual();
            } else if (gamepad2.b) {
                bot.gate.closeManual();
            } else {
                bot.gate.holdManual();
            }

            if (gamepad2.dpad_up) {
                isOuttaking = true;
                bot.canon.motor.setVelocity(bot.canon.CLOSE_SPEED);
            }
            if (gamepad2.dpad_down) {
                isOuttaking = false;
                bot.canon.motor.setPower(0);
            }
            if (gamepad2.dpad_right) {
                isOuttaking = false;
                bot.canon.motor.setPower(-bot.canon.CLOSE_SPEED);
            }
            if (gamepad2.y) {
                bot.canon.setPower(-1.0);
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

            double speed = (gamepad1.left_bumper) ? 0.8 : 1.0;

            bot.moveFieldCentric(x, y, rx, speed, Op.TELE);
            return true;
        }
    }
    //END OF MANUAL CONTROLS

    @Override
    public void runOpMode() {
        //Pose2d startPose = new Pose2d(60.1, -12.7, 0);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)); //The start position,
        // which is on the left tile of the far launch zone, with the intake up against the wall.

        bot = new Bot(hardwareMap, drive.localizer, side, telemetry);
        bot.initialize();

        TelemetryPacket t = new TelemetryPacket();

        Action defaultAction = new ParallelAction(
            new FieldCentricMovement(),
            new ManualControls()
        );
        Action currentAction = defaultAction;

        waitForStart();
        while (opModeIsActive()) {
            telemetry.update();
            telemetry.addData("LocalizerX: ", drive.localizer.getPose().position.x);
            telemetry.addData("LocalizerY: ", drive.localizer.getPose().position.y);
            telemetry.addData("LocalizerR: ", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("Using April Tag: ", useAprilTag);
            telemetry.addLine();

            bot.aprilTagTelementary();

            bot.updatePoseUsingAprilTag();

            drive.updatePoseEstimate();
            currentPose = drive.localizer.getPose();

            if (gamepad1.xWasPressed()) {
                currentAction = new SequentialAction(
                    new ParallelAction(
                        bot.moveTo(bot.launchPose),
                        bot.canon.spinUp(bot.canon.CLOSE_SPEED)
                    ),
                    new Wait(0.7),
                    bot.shootClose()
                );
            }

            /*if (gamepad1.aWasPressed()) {
                currentAction = new SequentialAction(
                    new ParallelAction(
                        bot.moveTo(bot.launchPose),
                        bot.canon.spinUp(bot.canon.CLOSE_SPEED)
                    ),
                    bot.shootCloseNew()
                );
            }*/

            if (gamepad1.dpadRightWasPressed()) {
                useAprilTag = !useAprilTag;
            }
            if (gamepad1.bWasPressed()) {
                bot.canon.setPower(0);
                bot.gate.closeManual();
                isOuttaking = false;
                currentAction = defaultAction;
            }

            if (!currentAction.run(t)) {
                currentAction = defaultAction;
            }

            telemetry.addData("Target Speed: ", bot.canon.CLOSE_SPEED);
            telemetry.addData("Canon power: ", bot.canon.motor.getPower());
            telemetry.addData("Canon speed: ", bot.canon.motor.getVelocity());
            telemetry.addData("Pos: ", currentPose);
        }
    }

    public Action pathToPos(Pose2d startPose, Pose2d target) {
        return drive.actionBuilder(startPose)
            .strafeToSplineHeading(target.position, target.heading.toDouble())
            .build();
    }
}

