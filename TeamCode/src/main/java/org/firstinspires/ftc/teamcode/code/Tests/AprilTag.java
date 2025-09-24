package org.firstinspires.ftc.teamcode.code.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AprilTag extends LinearOpMode {
    public void runOpMode() {
        AprilTagProcessor aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal myVisionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "camera"),
            aprilTagProcessor
        );

        while(opModeIsActive()) {
            ArrayList<AprilTagDetection> aprilTags = aprilTagProcessor.getDetections();
            for (int i=aprilTags.size()-1; i>=aprilTags.size()-5 && i>=0; i--) {
                telemetry.addData("Tag: ", aprilTags.get(i).id);
            }
            telemetry.update();
        }
    }
}
