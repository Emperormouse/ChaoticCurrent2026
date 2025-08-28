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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.code.utility.ImprovedGamepad;
import org.jetbrains.annotations.NotNull;

//This is a teleOp that I'm working on which will be able to run RoadRunner paths in teleOp.
//This will most likely be used if we want to make a TeleOp which is mostly automated
@TeleOp(name = "RRTeleOp")
public class RoadRunnerTeleOp extends LinearOpMode {
    private final DcMotor frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
    private final DcMotor backLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
    private final DcMotor frontRight = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
    private final DcMotor backRight = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");
    private final ImprovedGamepad pad1 = new ImprovedGamepad(gamepad1);

    //This is the roadrunner mecanum drive
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


    @Override
    public void runOpMode() {

        while(opModeIsActive()) {
            //TODO: Replace this with the actual current location found with the odometry system
            Pose2d currentPose = new Pose2d(0, 0, 0);

            //When a is pressed, this path will be run
            if (pad1.a.justPressed()) {
                path1(currentPose);
            }

            fieldCentricMovement(currentPose);
        }
    }

    //A roadrunner path that could be run during TeleOp
    public void path1(Pose2d startPose) {
        //A path which moves 30 inches in the x-direction and 10 in the y-direction, and then turns 90 degrees.
        Action path = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(30, 10))
                .turn(Math.toRadians(90.0))
                .build();

        TelemetryPacket t = new TelemetryPacket();
        //This will run the path in small steps, which should just be a fraction of a second each time,
        //and between those it will check if b is pressed or if a stop was requested on the driver hub,
        //and exit the path early if so.
        while(path.run(t)) { //path.run(t) both returns a boolean, and runs part of the path
            if (gamepad1.b || isStopRequested()) {
                return;
            }
        }
    }

    //This is code for field-centric movement, which means that the robot will move the same
    //way regardless of the direction that its facing. It's meant to be called many times in a loop
    private void fieldCentricMovement(Pose2d currentPose) {
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

        frontLeft.setPower(flPower / denominator);
        frontRight.setPower(frPower / denominator);
        backLeft.setPower(blPower / denominator);
        backRight.setPower(brPower / denominator);
    }
}

