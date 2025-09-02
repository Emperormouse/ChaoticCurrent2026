package org.firstinspires.ftc.teamcode.examples.Auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

//This is a more complex example which shows some more of the roadrunner methods for making paths.
//Strafing is the only method that you really need, but example 3 will show splines, which are a lot
//more complicated but are faster in many situations.
@Autonomous
public class ExampleAuto2 extends LinearOpMode {
    public void runOpMode() {
        //The robot starts at the bottom middle of the field facing into the center of teh field
        Pose2d startPose = new Pose2d(0, -60, toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //Watch the video first to see how this path looks
        Action path = drive.actionBuilder(startPose)
            //First the robot moves forward until it's y-position is -35
            .lineToY(-35)
            //It waits 2 seconds
            .waitSeconds(2.0)
            //It strafes to a few points, always facing forward
            .strafeTo(new Vector2d(25, -35))
            .strafeTo(new Vector2d(45, 0))
            .strafeTo(new Vector2d(45, -55))
            //It strafes forward a bit, but simultaneously turns so that it's facing -90 degrees
            .strafeToLinearHeading(new Vector2d(45, -40), toRadians(-90))
            //It drives forward a bit
            .lineToY(-55)
            //It waits 1.5 seconds
            .waitSeconds(1.5)
            //It then strafes back and forth, turning between 90 and -90 degrees each time
            .strafeToLinearHeading(new Vector2d(5, -35), toRadians(90))
            .waitSeconds(2.0)
            .strafeToLinearHeading(new Vector2d(45, -55), toRadians(-90))
            .waitSeconds(1.5)
            .strafeToLinearHeading(new Vector2d(10, -35), toRadians(90))
            .waitSeconds(2.0)
            .build();

        Actions.runBlocking(path);
    }
}
