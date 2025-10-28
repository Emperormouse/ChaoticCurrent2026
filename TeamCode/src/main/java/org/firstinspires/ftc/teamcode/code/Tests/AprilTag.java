package org.firstinspires.ftc.teamcode.code.Tests;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AprilTag extends LinearOpMode {
    enum Pattern {
        GPP,
        PGP,
        PPG
    }
    public void runOpMode() {
        AprilTagProcessor aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "camera"),
            aprilTagProcessor
        );

        Pattern pattern = Pattern.GPP;
        while(!opModeIsActive()) {
            ArrayList<AprilTagDetection> aprilTags = aprilTagProcessor.getDetections();

            switch(aprilTags.get(aprilTags.size()-1).id) {
                case 21: pattern = Pattern.GPP; break;
                case 22: pattern = Pattern.PGP; break;
                case 23: pattern = Pattern.PPG; break;
            }

            telemetry.addData("Number: ", aprilTags.get(aprilTags.size()-1).id);
            telemetry.addData("Pattern: ", pattern);
            telemetry.addData("Size: ", aprilTags.size());
            telemetry.update();
        }

        Pose2d startPos = new Pose2d(61, -20, toRadians(180));
        Vector2d launchVec = new Vector2d(55, -15);
        double launchHeading = toRadians(200);
        Pose2d launchPose = new Pose2d(launchVec.x, launchVec.y, launchHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);

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

        switch (pattern) {
            case GPP: Actions.runBlocking(gpp); break;
            case PGP: Actions.runBlocking(pgp); break;
            case PPG: Actions.runBlocking(ppg); break;
        }
    }
}
