package org.firstinspires.ftc.teamcode.examples.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//A super simple roadrunner auto to show the structure of it
@Autonomous
public class ExampleAuto1 extends LinearOpMode {
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //A path which moves 30 inches in the x-direction and 10 in the y-direction, and then turns 90 degrees.
        Action path = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(30, 10))
                .turn(Math.toRadians(90.0))
                .build();

        Actions.runBlocking(path);
    }
}
