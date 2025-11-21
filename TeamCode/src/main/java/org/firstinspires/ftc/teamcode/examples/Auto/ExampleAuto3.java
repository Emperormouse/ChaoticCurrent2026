package org.firstinspires.ftc.teamcode.examples.Auto;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class ExampleAuto3 extends LinearOpMode {
    public void runOpMode() {
        //The robot starts at the bottom middle of the field facing into the center of teh field
        Pose2d startPose = new Pose2d(0, -60, toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //Watch the video first to see how this path looks
        Action path = drive.actionBuilder(new Pose2d(0, -60, PI/2))
            //Hook first specimen
            .lineToY(-35)
            .waitSeconds(2.0)

            //Push third specimen into observation zone
            .setTangent(0)
            .splineToConstantHeading(new Vector2d(20, -35), 0)
            .splineToConstantHeading(new Vector2d(30, -20), toRadians(90))
            .splineToConstantHeading(new Vector2d(37, -10), 0)
            .setTangent(0)
            .splineToConstantHeading(new Vector2d(45, -30), toRadians(-90))
            .lineToY(-55)

            //Push fourth specimen into observation zone
            .lineToYSplineHeading(-30, PI/2)
            .splineToConstantHeading(new Vector2d(50, -10), 0)
            .splineToConstantHeading(new Vector2d(55, -20), toRadians(-90))
            .lineToY(-55)

            //Grab and hook second specimen
            .strafeToLinearHeading(new Vector2d(45, -45), toRadians(-90))
            .strafeTo(new Vector2d(45, -60))
            .waitSeconds(2.0)
            .strafeToLinearHeading(new Vector2d(5, -35), toRadians(90))
            .waitSeconds(2.0)

            //Grab and hook third
            .strafeToSplineHeading(new Vector2d(45, -60), toRadians(-90))
            .waitSeconds(2.0)
            .strafeToLinearHeading(new Vector2d(-5, -35), toRadians(90))
            .build();

        Actions.runBlocking(path);
    }
}
