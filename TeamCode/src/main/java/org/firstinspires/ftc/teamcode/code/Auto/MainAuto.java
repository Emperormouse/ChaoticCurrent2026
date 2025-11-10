package org.firstinspires.ftc.teamcode.code.Auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.code.Subsystems.Bot;

@Autonomous
public class MainAuto extends LinearOpMode {
    MecanumDrive drive;
    Bot bot;

    Vector2d launchVec = new Vector2d(55, -15);
    double launchHeading = toRadians(20);
    Pose2d launchPose = new Pose2d(launchVec.x, launchVec.y, launchHeading);

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        bot = new Bot(hardwareMap, drive.localizer, telemetry);

        bot.initialize();
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + 5000) { //Scan for april tag for 5 seconds
            bot.updatePoseUsingAprilTag();
            bot.aprilTagTelementary();
            telemetry.update();
        }

        Pose2d startPos = drive.localizer.getPose();
        Action pgp = drive.actionBuilder(startPos)
            .waitSeconds(2)
            .strafeToLinearHeading(launchVec, launchHeading)
            .waitSeconds(3)
            .setTangent(toRadians(180))
            .splineToLinearHeading(new Pose2d(35, -50, toRadians(-90)), toRadians(-90))
            .strafeToLinearHeading(launchVec, launchHeading)
            .waitSeconds(3)

            .splineToLinearHeading(new Pose2d(12, -50, toRadians(-90)), toRadians(-90))
            .strafeToLinearHeading(new Vector2d(0, -60), toRadians(180))
            .waitSeconds(3)
            .strafeToLinearHeading(launchVec, launchHeading)
            .waitSeconds(3)
            .build();

        Action gpp = drive.actionBuilder(startPos)
            .waitSeconds(2)
            .strafeToLinearHeading(launchVec, launchHeading)
            .waitSeconds(4)
            .setTangent(toRadians(180))
            .splineToLinearHeading(new Pose2d(12, -40, toRadians(-90)), toRadians(-90))
            .lineToY(-50)
            .strafeToLinearHeading(launchVec, launchHeading)
            .waitSeconds(3)

            .splineToLinearHeading(new Pose2d(36, -40, toRadians(-90)), toRadians(-90))
            .lineToY(-50)
            .strafeToLinearHeading(new Vector2d(0, -60), toRadians(180))
            .waitSeconds(3)
            .strafeToLinearHeading(launchVec, launchHeading)
            .waitSeconds(3)
            .build();

        Action ppg = drive.actionBuilder(startPos)
            .waitSeconds(2)
            .strafeToLinearHeading(launchVec, launchHeading)
            .waitSeconds(4)

            .setTangent(toRadians(180))
            .splineToSplineHeading(new Pose2d(12, -50, toRadians(-90)), toRadians(-90))
            .setTangent(toRadians(90))
            .waitSeconds(0.5)
            .splineToLinearHeading(launchPose, 0)
            .waitSeconds(4)

            .splineToLinearHeading(new Pose2d(7, -54, 0), toRadians(-90))
            .waitSeconds(2)
            .lineToX(23)
            .setTangent(0)
            .splineToSplineHeading(new Pose2d(33, -40, toRadians(100)), toRadians(90))
            .setTangent(toRadians(90))
            .splineToSplineHeading(launchPose, toRadians(45))
            .waitSeconds(4)
            .lineToX(launchVec.x+1)
            .build();

        Action chosenPath = pgp;
        waitForStart();

        //END OF INIT

        Actions.runBlocking(chosenPath);
    }
}
