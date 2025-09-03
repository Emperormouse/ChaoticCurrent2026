package org.firstinspires.ftc.teamcode.examples.Auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.examples.subsystems.Slides;

@Autonomous
public class ExampleAuto4 extends LinearOpMode {
    public void runOpMode() {

        Pose2d startPose = new Pose2d(0, -60, toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Slides slides = new Slides(hardwareMap);


        Action path = drive.actionBuilder(startPose)
            .lineToY(-35)
            .stopAndAdd(slides.moveSlides(400))
            .waitSeconds(2.0)
            .stopAndAdd(slides.moveSlides(0))

            .strafeTo(new Vector2d(25, -35))
            .strafeTo(new Vector2d(45, 0))
            .strafeTo(new Vector2d(45, -55))

            //This action starts running asynchronously
            .afterTime(0, slides.moveSlides(300))
            .strafeToLinearHeading(new Vector2d(45, -40), toRadians(-90))

            .lineToY(-55)

            .waitSeconds(1.5)
            .stopAndAdd(slides.moveSlides(0))

            .afterTime(1, slides.moveSlides(400))
            .strafeToLinearHeading(new Vector2d(5, -35), toRadians(90))
            .waitSeconds(1.5)
            .stopAndAdd(slides.moveSlides(0))

            .afterTime(1, slides.moveSlides(300))
            .strafeToLinearHeading(new Vector2d(45, -55), toRadians(-90))
            .waitSeconds(1.5)
            .stopAndAdd(slides.moveSlides(0))

            .afterTime(1, slides.moveSlides(400))
            .strafeToLinearHeading(new Vector2d(5, -35), toRadians(90))
            .waitSeconds(1.5)
            .stopAndAdd(slides.moveSlides(0))
            .build();

        Actions.runBlocking(path);
    }
}
