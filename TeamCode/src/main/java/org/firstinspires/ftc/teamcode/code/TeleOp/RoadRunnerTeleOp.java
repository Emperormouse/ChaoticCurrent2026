package org.firstinspires.ftc.teamcode.code.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.code.Subsystems.Canon;
import org.firstinspires.ftc.teamcode.code.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.code.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.code.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterFirstParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;

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
    private Canon canon;
    private Intake intake;
    private Outtake outtake;
    private Gate gate;
    private Pose2d currentPose;
    private final int CANON_SPEED = -1200;

    //This is the roadrunner mecanum drive
    //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

    //MANUAL CONTROLS BESIDES MOVEMENT IN HERE
    private class ManualControls implements Action {
        public boolean run(@NonNull TelemetryPacket t) {

            if (gamepad1.a) {
                telemetry.addLine("A pressed");
            }

            if (gamepad1.left_bumper) {
                intake.intake();
            } else if (gamepad1.right_bumper) {
                intake.reverse();
            } else {
                intake.stop();
            }

            if (gamepad1.a) {
                gate.openManual();
            }
            if (gamepad1.b) {
                gate.closeManual();
            }

            return true;
        }
    }

    //This is code for field-centric movement, which means that the robot will move the same
    //way regardless of the direction that its facing. It's meant to be called many times in a loop
    //It's structured as an Action however since this teleOp is based on roadrunner Actions
    private class FieldCentricMovement implements Action {
        public boolean run(@NonNull TelemetryPacket t) {
            double botRot = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            //double botRot = 0;

            double frPower = 0;
            double flPower = 0;
            double brPower = 0;
            double blPower = 0;

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            frPower += rx;
            brPower += rx;
            flPower -= rx;
            blPower -= rx;

            //FORWARD-DIRECTION
            frPower += y * cos(botRot);
            brPower += y * cos(botRot);
            flPower += y * cos(botRot);
            blPower += y * cos(botRot);

            frPower -= y * sin(botRot);
            brPower += y * sin(botRot);
            flPower -= y * sin(botRot);
            blPower += y * sin(botRot);

            //SIDEWAYS-DIRECTION
            frPower += x * sin(botRot);
            brPower += x * sin(botRot);
            flPower += x * sin(botRot);
            blPower += x * sin(botRot);

            frPower += x * cos(botRot);
            brPower -= x * cos(botRot);
            flPower += x * cos(botRot);
            blPower -= x * cos(botRot);

            double denominator = max(1, max(max(max(abs(frPower), abs(brPower)), abs(flPower)), abs(blPower)));

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

        canon = new Canon(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        outtake = new Outtake(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        ));
        imu.resetYaw();

        //FtcDashboard dash = FtcDashboard.getInstance();
        TelemetryPacket t = new TelemetryPacket();

        Action defaultAction = new ParallelAction(
            new FieldCentricMovement(),
            new ManualControls()
        );
        /*Action currentAction = new ParallelAction(
            defaultAction,
            canon.maintainSpeed(CANON_SPEED)
        );*/
        Action currentAction = defaultAction;

        waitForStart();
        while (opModeIsActive()) {
            //TODO: Replace this with the actual current location found with the odometry system
            currentPose = new Pose2d(0, 0, 0);
            //dash.sendTelemetryPacket(t);
            telemetry.update();

            if (gamepad1.dpadUpWasPressed()) {
                if (Math.abs(canon.motor.getVelocity()) < 100) {
                    currentAction = new ParallelAction(
                        defaultAction,
                        canon.maintainSpeed(CANON_SPEED)
                    );
                } else {
                    canon.setPower(0);
                    currentAction = defaultAction;
                }
            }
            if (gamepad1.xWasPressed()) {
                currentAction = new ParallelAction(
                    shootFar(),
                    new FieldCentricMovement()
                );
            }

            if (!currentAction.run(t)) {
                currentAction = defaultAction;
            }

            telemetry.addData("Canon power: ", canon.motor.getPower());
            telemetry.addData("Canon speed: ", canon.motor.getVelocity());
            telemetry.addData("Rotation: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }
    }

    public Action shootFar() {
        return new SequentialAction(
            gate.open(),
            canon.spinUp(CANON_SPEED),
            new EndAfterFirstParallel (
                new SequentialAction(
                    intake.spinForDuration(1.0),
                    new Wait(2.0),
                    intake.spinForDuration(1.0),
                    new Wait(2.0),
                    intake.spinForDuration(1.0),
                    new Wait(2.0)
                ),
                canon.maintainSpeed(CANON_SPEED)
            ),
            gate.close()
        );
    }

    //A roadrunner path
    /*public Action path1(Pose2d startPose) {
        //A path which moves to the point
        return drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(20, 10))
                .turn(Math.toRadians(90.0))
                .build();
    }

    //A roadrunner path
    public Action path2(Pose2d startPose) {
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

