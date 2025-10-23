package org.firstinspires.ftc.teamcode.code.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.code.Subsystems.Bot;
import org.firstinspires.ftc.teamcode.code.Subsystems.Canon;
import org.firstinspires.ftc.teamcode.code.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.code.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterFirstParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;

import java.util.Arrays;

//This is a teleOp that I'm working on which will be able to run RoadRunner paths in teleOp.
//The best way that I figured out to do this is to make every part of the teleOp an Action, which
//is what roadrunner paths are, and then switch between those actions.
//This will most likely be used if we want to make a TeleOp which is mostly automated
@TeleOp(name = "RoadRunnerTeleOp")
public class RoadRunnerTeleOp extends LinearOpMode {
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

            if (gamepad1.dpadLeftWasPressed()) {
                bot.canon.FAR_SPEED += 20;
            }
            if (gamepad1.dpadRightWasPressed()) {
                bot.canon.FAR_SPEED -= 20;
            }

            return true;
        }
    }

    //This is code for field-centric movement, which means that the robot will move the same
    //way regardless of the direction that its facing. It's meant to be called many times in a loop
    //It's structured as an Action however since this teleOp is based on roadrunner Actions
    private class FieldCentricMovement implements Action {
        public boolean run(@NonNull TelemetryPacket t) {
            double botRot = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - Math.PI/2;
            //double botRot = 0;

            double frPower = 0;
            double flPower = 0;
            double brPower = 0;
            double blPower = 0;

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;
            double rx2 = -gamepad2.right_stick_x;

            if (isOuttaking) {
                frPower += rx/3;
                brPower += rx/3;
                flPower -= rx/3;
                blPower -= rx/3;
            } else {
                frPower += rx;
                brPower += rx;
                flPower -= rx;
                blPower -= rx;
            }

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
            if (gamepad1.left_bumper) {
                denominator *= (5/3.0);
            }


            frontLeft.setPower(flPower / denominator);
            frontRight.setPower(frPower / denominator);
            backLeft.setPower(blPower / denominator);
            backRight.setPower(brPower / denominator);

            return true;
        }
    }
    //END OF MANUAL CONTROLS

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        bot = new Bot(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        ));
        imu.resetYaw();

        drive = new MecanumDrive(hardwareMap, new Pose2d(60.1, -12.7, 0)); //The start position,
        // which is on the left tile of the far launch zone, with the intake up against the wall.

        TelemetryPacket t = new TelemetryPacket();

        Actions.runBlocking(new SequentialAction(
            bot.canon.spinUp(bot.canon.CLOSE_SPEED),
            new EndAfterFirstParallel(
                new Wait(4.0),
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
            drive.updatePoseEstimate();
            currentPose = drive.localizer.getPose();
            //dash.sendTelemetryPacket(t);
            telemetry.update();

            /*if (Math.abs(bot.canon.motor.getVelocity() - bot.canon.CLOSE_SPEED) <= 20) {
                gamepad1.setLedColor(0, 255, 0, 1);
            } else {
                gamepad1.setLedColor(255, 0, 0, 1);
            }*/

            if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
                if (!isOuttaking) {
                    isOuttaking = true;
                    bot.canon.setPower(bot.canon.closePower);
                } else {
                    isOuttaking = false;
                    bot.canon.setPower(0);
                    bot.gate.close();
                }
            }
            if (gamepad1.xWasPressed()) {
                currentAction = pathToLaunchPosClose(currentPose);
            }
            if (gamepad1.yWasPressed()) {
                currentAction = pathToLaunchPos(currentPose);
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
            .strafeToSplineHeading(new Vector2d(-20, -20), Math.toRadians(45))
            .build();
    }

    //A roadrunner path
    /*public Action path2(Pose2d startPose) {
        //A path which moves to the point
        return drive.actionBuilder(startPose)
            .strafeTo(new Vector2d(50, 20))
            .turn(Math.toRadians(360.0))
            .turn(Math.toRadians(-360))
            .turn(Math.toRadians(360.0))
            .turn(Math.toRadians(-360))
            .turn(Math.toRadians(360.0))
            .turn(Math.toRadians(-360))
            .turn(Math.toRadians(360.0))
            .turn(Math.toRadians(-360))
            .build();
    }*/



}

