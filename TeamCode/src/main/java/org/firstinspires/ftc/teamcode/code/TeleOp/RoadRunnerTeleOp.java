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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.code.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.code.Subsystems.Outtake;

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
    //private final IMU imu = hardwareMap.get(IMU.class, "imu");
    private Outtake outtake;
    private Intake intake;
    private Pose2d currentPose;

    //This is the roadrunner mecanum drive
    //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

    //MANUAL CONTROLS BESIDES MOVEMENT IN HERE
    private class ManualControls implements Action {
        public boolean run(@NonNull TelemetryPacket t) {

            if (gamepad1.a) {
                telemetry.addLine("A pressed");
            }

            if (gamepad1.dpad_up) {
                intake.startIntake();
            }
            if (gamepad1.dpad_down) {
                intake.reverse();
            }
            if (gamepad1.dpad_right) {
                intake.stop();
            }

            return true;
        }
    }

    //This is code for field-centric movement, which means that the robot will move the same
    //way regardless of the direction that its facing. It's meant to be called many times in a loop
    //It's structured as an Action however since this teleOp is based on roadrunner Actions
    private class FieldCentricMovement implements Action {
        public boolean run(@NonNull TelemetryPacket t) {
            //double botRot = imu.getRobotYawPitchRollAngles().getYaw(); //RADIANS
            double botRot = 0;
            telemetry.addData("Heading: ", botRot);

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

            //X-DIRECTION
            frPower += y * cos(botRot);
            brPower += y * cos(botRot);
            flPower += y * cos(botRot);
            blPower += y * cos(botRot);

            frPower -= y * sin(botRot);
            brPower += y * sin(botRot);
            flPower += y * sin(botRot);
            blPower -= y * sin(botRot);

            //Y-DIRECTION
            frPower += x * sin(botRot);
            brPower += x * sin(botRot);
            flPower += x * sin(botRot);
            blPower += x * sin(botRot);

            frPower += x * cos(botRot);
            brPower -= x * cos(botRot);
            flPower -= x * cos(botRot);
            blPower += x * cos(botRot);

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
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);

        //FtcDashboard dash = FtcDashboard.getInstance();
        TelemetryPacket t = new TelemetryPacket();

        Action defaultAction = new ParallelAction(
            new FieldCentricMovement(),
            new ManualControls()
        );
        Action currentAction = defaultAction;

        waitForStart();
        while (opModeIsActive()) {
            //TODO: Replace this with the actual current location found with the odometry system
            currentPose = new Pose2d(0, 0, 0);
            //dash.sendTelemetryPacket(t);
            telemetry.update();

            //All of the controls for switching between roadrunner paths should go here.
            //These controls can always be accessed regardless of which action is currently running
            //Always use "WasPressed" controls for changing the running action
            if (gamepad1.bWasPressed()) { //Ends any running paths/actions and returns to manual mode
                currentAction = defaultAction;
            }
            if (gamepad1.aWasPressed()) {
                currentAction = new ParallelAction(
                    defaultAction,
                    outtake.shootFar()
                );
            }

            if (!currentAction.run(t)) {
                currentAction = defaultAction;
            }

            telemetry.addData("Canon powerR: ", outtake.canon.motorR.getPower());
            telemetry.addData("Canon powerL: ", outtake.canon.motorL.getPower());
        }


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

