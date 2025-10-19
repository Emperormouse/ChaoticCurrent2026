package org.firstinspires.ftc.teamcode.code.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class MainTeleOp extends LinearOpMode {
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "back_right");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        while(opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            double botRot = pose.heading.toDouble();
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
                denominator *= 2;
            }

            frontLeft.setPower(flPower / denominator);
            frontRight.setPower(frPower / denominator);
            backLeft.setPower(blPower / denominator);
            backRight.setPower(brPower / denominator);

            telemetry.addData("pos: ", pose.position);
            telemetry.addData("r: ", pose.heading);
            telemetry.update();
        }
    }
}
