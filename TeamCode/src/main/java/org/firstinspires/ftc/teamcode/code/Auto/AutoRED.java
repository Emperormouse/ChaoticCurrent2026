package org.firstinspires.ftc.teamcode.code.Auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.code.Subsystems.Bot;
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterEitherParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.KeepRunning;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;
import org.firstinspires.ftc.teamcode.code.utility.Op;
import org.firstinspires.ftc.teamcode.code.utility.Side;

@Autonomous
public class AutoRED extends LinearOpMode {
    MecanumDrive drive;
    Bot bot;

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-56.86, 47, Math.toRadians(-56.25)));
        bot = new Bot(hardwareMap, drive.localizer, Side.RED, telemetry);

        bot.initialize();
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + 5000) { //Scan for april tag for 5 seconds
            bot.updatePoseUsingAprilTag();
            bot.aprilTagTelementary();
            telemetry.update();
        }

        Pose2d startPos = drive.localizer.getPose();
        telemetry.addData("Start Position: ", startPos);
        telemetry.update();

        Action pgp = new SequentialAction(
            bot.moveTo(bot.launchPose),
            new Wait(0.5),
            bot.shootClose(Op.AUTO),

            bot.intake.setPower(-1.0),
            bot.moveToImprecise(new Pose2d(-11, 14, toRadians(90))),
            new EndAfterEitherParallel(
                new Wait(2.2),
                bot.moveToImprecise(new Pose2d(-11, 55, toRadians(90)), 1.0)
            ),
            new Wait(0.6),
            bot.intake.setPower(0),

            bot.moveTo(bot.launchPose),
            bot.shootClose(Op.AUTO),

            bot.intake.setPower(-1.0),
            bot.moveToImprecise(new Pose2d(10.5, 12, toRadians(90))),
            new EndAfterEitherParallel(
                new Wait(3.0),
                bot.moveToImprecise(new Pose2d(10.5, 62, toRadians(90)), 1.0)
            ),
            new Wait(0.7),
            bot.moveToImprecise(new Pose2d(10, 40, toRadians(90))),
            bot.intake.setPower(0),

            bot.moveTo(bot.launchPose),
            bot.shootClose(Op.AUTO),
            bot.moveToImprecise(new Pose2d(-35, 14.8, Math.toRadians(-41)))
        );


        /*Action gpp = drive.actionBuilder(startPos)
            //.waitSeconds(2)
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
            //.waitSeconds(3)
            .build();

        Action ppg = drive.actionBuilder(startPos)
            //.waitSeconds(2)
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
            //.waitSeconds(4)
            //.lineToX(launchVec.x+1)
            .build();*/

        Action chosenPath = pgp;
        waitForStart();

        //END OF INIT

        Actions.runBlocking(new ParallelAction(
                chosenPath,
                bot.updatePoseUsingAprilTagAction(),
                new KeepRunning(bot.canon.setVelInstant(bot.canon.CLOSE_SPEED))
            )
        );
    }
}
