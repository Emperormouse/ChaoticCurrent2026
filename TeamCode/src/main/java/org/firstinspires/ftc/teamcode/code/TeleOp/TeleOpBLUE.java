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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.code.Subsystems.Bot;
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterFirstParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.KeepRunning;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;
import org.firstinspires.ftc.teamcode.code.utility.Op;
import org.firstinspires.ftc.teamcode.code.utility.Side;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class TeleOpBLUE extends LinearOpMode {
    private Bot bot;
    private Pose2d currentPose;
    private boolean useAprilTag = false;
    Pose2d launchPose = new Pose2d(-24.7, -17, Math.toRadians(50));
    private double distance = 0.0;
    private AprilTagDetection latestAprilTagDetection = null;

    Side side = Side.BLUE;

    //This is the roadrunner mecanum drive
    MecanumDrive drive;

    double lastRightTrigger = 0.0;
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
                if (gamepad1.right_trigger > 0.1) {
                    bot.gate.openManual();
                    if (distance > 70)
                        bot.intake.setPowerManual(-0.8);
                    else
                        bot.intake.setPowerManual(-1.0);
                } else if (lastRightTrigger > 0.1) {
                    bot.gate.closeManual();
                    bot.intake.stop();
                } else {
                    bot.intake.stop();
                }
            }
            lastRightTrigger = gamepad1.right_trigger;

            if (Math.abs(gamepad1.left_trigger) > 0.2) {
                bot.canon.motor.setVelocity(bot.canon.CLOSE_SPEED);
            }

            if (gamepad2.a) {
                bot.canon.motor.setPower(1.0);
            } else if (gamepad2.b) {
                bot.canon.motor2.setPower(1.0);
            } else if (gamepad2.y) {
                bot.canon.motor.setPower(1.0);
                bot.canon.motor2.setPower(1.0);
            }

            if (gamepad2.dpadUpWasPressed()) {
                bot.canon.b -= 10;
            }
            if (gamepad2.dpadDownWasPressed()) {
                bot.canon.b += 10;
            }
            if (gamepad2.y) {
                bot.canon.motor.setVelocity(bot.canon.CLOSE_SPEED);
            }

            if (gamepad2.dpadRightWasPressed()) {
                bot.targetDistance += 1;
            }
            if (gamepad2.dpadLeftWasPressed()) {
                bot.targetDistance -= 1;
            }

            /*if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
                bot.canon.motor.setVelocity(-1850);
            }*/


            return true;
        }
    }

    @Override
    public void runOpMode() {
        //Pose2d startPose = new Pose2d(60.1, -12.7, 0);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)); //The start position,
        // which is on the left tile of the far launch zone, with the intake up against the wall.

        bot = new Bot(hardwareMap, drive, side, telemetry);
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

            /*Vector2d aprilVec = new Vector2d(-58.3727f, -55.6425f);

            Pose2d botPose = bot.localizer.getPose();
            double dx = botPose.position.x - aprilVec.x;
            double dy = botPose.position.y - aprilVec.y;
            latestAprilTagDetection = bot.getLatestAprilTagDetection();
            if (latestAprilTagDetection != null) {
                distance = latestAprilTagDetection.ftcPose.y;
            } else {
                distance = Math.sqrt(dx*dx + dy*dy);
            }

            telemetry.addData("Distance: ", distance);
            if (latestAprilTagDetection != null)
                telemetry.addData("April Center: ", latestAprilTagDetection.center.x);

            telemetry.addData("Target Distance: ", bot.targetDistance);
            telemetry.addData("Target Speed: ", bot.canon.CLOSE_SPEED);
            telemetry.addData("Target Speed First: ", bot.canon.CLOSE_SPEED_FIRST);

            telemetry.addData("LocalizerX: ", drive.localizer.getPose().position.x);
            telemetry.addData("LocalizerY: ", drive.localizer.getPose().position.y);
            telemetry.addData("LocalizerR: ", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("Using April Tag: ", useAprilTag);
            telemetry.addLine();

            bot.aprilTagTelementary();

            bot.updatePoseUsingAprilTag();

            drive.updatePoseEstimate();
            currentPose = drive.localizer.getPose();*/

            if (gamepad1.xWasPressed()) {
                currentAction = new SequentialAction(
                    bot.moveToContinuous(new Pose2d(-0.5, -50, 0)),
                    bot.moveTo(new Pose2d(-0.5, -54, 0))
                );
            }

            if (gamepad1.dpadUpWasPressed()) {
                currentAction = bot.moveTo(new Pose2d(25.1,29.4,0));
            }

            /*if (gamepad1.aWasPressed()) {
                bot.intake.stop();
                bot.gate.close();
                currentAction = new ParallelAction(
                    new TrackedMovement(),
                    new ManualControls(),
                    bot.canon.setVelByDistance()
                );
            }*/

            if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
                bot.canon.setPower(0);
                bot.gate.closeManual();
                currentAction = defaultAction;
            }

            if (!currentAction.run(t)) {
                currentAction = defaultAction;
            }
            Actions.runBlocking(bot.canon.cloneMotorPower());


            telemetry.addData("Canon power: ", bot.canon.motor.getPower());
            telemetry.addData("Canon speed: ", bot.canon.motor.getVelocity());
            telemetry.addData("Pos: ", currentPose);
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

            double speed = (Math.abs(gamepad1.left_trigger) > 0.2) ? 0.33 : 1.0;

            bot.moveFieldCentric(x, y, rx, speed, Op.TELE);

            return true;
        }
    }

    private class TrackedMovement implements Action {
        public boolean run(@NonNull TelemetryPacket t) {
            double kr2 = (1.0 / 60);
            Vector2d aprilVec = new Vector2d(-58.3727f, -55.6425f);
            Vector2d goalVec = new Vector2d(aprilVec.x-7, aprilVec.y-7);

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;

            Pose2d botPose = bot.localizer.getPose();
            double dx = botPose.position.x - goalVec.x;
            double dy = botPose.position.y - goalVec.y;

            double targetAngle = Math.atan2(dx, -dy) - Math.PI/2;
            if (targetAngle < -Math.PI) {
                targetAngle += 2*Math.PI;
            }

            double angleDiff = targetAngle - botPose.heading.toDouble();
            double r = Math.toDegrees(angleDiff) * kr2;

            bot.moveFieldCentric(x, y, r, Op.TELE);

            return true;
        }
    }
}

