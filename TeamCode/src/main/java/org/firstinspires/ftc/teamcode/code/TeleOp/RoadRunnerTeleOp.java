package org.firstinspires.ftc.teamcode.code.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.code.Subsystems.Canon;

//This is a teleOp that I'm working on which will be able to run RoadRunner paths in teleOp.
//The best way that I figured out to do this is to make every part of the teleOp an Action, which
//is what roadrunner paths are, and then switch between those actions.
//This will most likely be used if we want to make a TeleOp which is mostly automated
@TeleOp(name = "RRTeleOp")
public class RoadRunnerTeleOp extends LinearOpMode {
    //private final DcMotor frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
    //private final DcMotor backLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
    //private final DcMotor frontRight = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
    //private final DcMotor backRight = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");
    private final Canon canon = new Canon(hardwareMap);
    private Pose2d currentPose;

    //This is the roadrunner mecanum drive
    //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

    //MANUAL CONTROLS BESIDES MOVEMENT IN HERE
    private class ManualControls implements Action {
        public boolean run(@NonNull TelemetryPacket t) {
            if (gamepad1.a) {
                //do something
            }

            return true;
        }
    }

    //This is code for field-centric movement, which means that the robot will move the same
    //way regardless of the direction that its facing. It's meant to be called many times in a loop
    //It's structured as an Action however since this teleOp is based on roadrunner Actions
    private class FieldCentricMovement implements Action {
        public boolean run(@NonNull TelemetryPacket t) {
            double botRot = currentPose.heading.toDouble(); //RADIANS

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

            //frontLeft.setPower(flPower / denominator);
            //frontRight.setPower(frPower / denominator);
            //backLeft.setPower(blPower / denominator);
            //backRight.setPower(brPower / denominator);

            return true;
        }
    }
    //END OF MANUAL CONTROLS

    @Override
    public void runOpMode() {
        //FtcDashboard dash = FtcDashboard.getInstance();
        TelemetryPacket t = new TelemetryPacket();

        Action defaultAction = canon.maintainSpeed(1500); /*new ParallelAction(
            new FieldCentricMovement(),
            new ManualControls()
        );*/
        Action currentAction = null;
        boolean isActionRunning = false;

        while (opModeIsActive()) {
            //TODO: Replace this with the actual current location found with the odometry system
            currentPose = new Pose2d(0, 0, 0);
            //dash.sendTelemetryPacket(t);
            telemetry.update();

            //All of the controls for switching between roadrunner paths should go here.
            //These controls can always be accessed regardless of which action is currently running
            //Always use "WasPressed" controls for changing the running action
            if (gamepad1.bWasPressed()) { //Ends any running paths/actions and returns to manual mode
                isActionRunning = false;
            }
            /*if (gamepad1.dpadUpWasPressed()) {
                currentAction = path1(currentPose);
                isActionRunning = true;
            }
            if (gamepad1.dpadRightWasPressed()) {
                currentAction = path2(currentPose);
                isActionRunning = true;
            }*/

            if (isActionRunning) {
                isActionRunning = currentAction.run(t);
            } else {
                defaultAction.run(t);
            }
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

