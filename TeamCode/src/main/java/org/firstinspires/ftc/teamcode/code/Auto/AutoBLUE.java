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
public class AutoBLUE extends LinearOpMode {
    MecanumDrive drive;
    Bot bot;

    public void waitSeconds(double time) {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + time*1000);
    }

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-49.5, -50, Math.toRadians(56)));
        bot = new Bot(hardwareMap, drive.localizer, Side.BLUE, telemetry);

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
            new EndAfterEitherParallel(
                new Wait(5.5),
                bot.moveTo(bot.launchPose)
            ),
            new Wait(0.5),
            bot.shootClose(Op.AUTO),

            /*drive.actionBuilder(launchPose)
                //.setTangent(toRadians(180))
                .strafeToSplineHeading(new Vector2d(39, -15), toRadians(-90))
                .strafeToSplineHeading(new Vector2d(39, -45), toRadians(-90))
                .strafeToSplineHeading(new Vector2d(30, -15), toRadians(30))
                .build(),*/
            bot.intake.setPower(-1.0),
            bot.moveToImprecise(new Pose2d(-11, -12, toRadians(-85))),
            new EndAfterEitherParallel(
                new Wait(2.3),
                bot.moveToImprecise(new Pose2d(-11, -53.5, toRadians(-85)), 1.0)
            ),
            new Wait(0.8),
            /*drive.actionBuilder(launchPose)
                //.setTangent(toRadians(180))
                .strafeToLinearHeading(new Vector2d(-11, -12), toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-11, -50), toRadians(-90))
                .build(),*/
            bot.intake.setPower(0),

            new EndAfterEitherParallel(
                new Wait(4.5),
                bot.moveTo(bot.launchPose)
            ),
            bot.shootClose(Op.AUTO),

            bot.intake.setPower(-1.0),
            bot.moveToImprecise(new Pose2d(13, -12, toRadians(-85))),
            new EndAfterEitherParallel(
                new Wait(2.8),
                bot.moveToImprecise(new Pose2d(13, -57.5, toRadians(-85)), 1.0)
            ),
            new Wait(0.5),
            bot.moveToImprecise(new Pose2d(13, -40, toRadians(-85))),
            /*drive.actionBuilder(launchPose)
                .strafeToLinearHeading(new Vector2d(13, -12), toRadians(-90))
                .strafeToLinearHeading(new Vector2d(13, -50), toRadians(-90))
                .build(),*/
            bot.intake.setPower(0),

            new EndAfterEitherParallel(
                new Wait(4.5),
                bot.moveTo(bot.launchPose)
            ),
            bot.shootClose(Op.AUTO),
            bot.moveTo(new Pose2d(-34.3, -9.8, Math.toRadians(54)))
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
