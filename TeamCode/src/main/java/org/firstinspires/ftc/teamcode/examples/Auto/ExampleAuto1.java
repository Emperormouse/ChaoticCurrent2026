package org.firstinspires.ftc.teamcode.examples.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//This example is mostly meant to show the structure of an auto opMode using roadrunner
//There is no loop, just a path which is created from multiple steps and then run.
@Autonomous
public class ExampleAuto1 extends LinearOpMode {
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //This part is the path
        //Don't worry about what this path does, the specifics of making paths will be explained
        //in the next example
        Action path = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(30, 10))
                .turn(Math.toRadians(90.0))
                .build();

        Actions.runBlocking(path);
    }
}
