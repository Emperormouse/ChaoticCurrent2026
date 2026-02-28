package org.firstinspires.ftc.teamcode.code.Auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.code.Subsystems.Bot;
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterEitherParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterFirstParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.KeepRunning;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;
import org.firstinspires.ftc.teamcode.code.utility.Op;
import org.firstinspires.ftc.teamcode.code.utility.Side;

@Autonomous
@Config
public class AutoBLUE extends LinearOpMode {
    public static class PARAMS {
        public double x1 = 8.3;
        public double angle = -120;
        public double y1 = -50;
        public double y2 = -62.8;
        public double xMiddle = 10;
        public double yMiddle1 = -35;
        public double yMiddle2 = -59;
        public double closeX = -13.5;
        public double closeY1 = -33;
        public double closeY2 = -53;

    }
    public static PARAMS PARAMS = new PARAMS();

    MecanumDrive drive;
    Bot bot;
    Pose2d launchPose = new Pose2d(-14, -16, Math.toRadians(38));
    Vector2d launchVec = new Vector2d(launchPose.position.x, launchPose.position.y);
    double launchSpeed = 1420;

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-69, -44.44, Math.toRadians(41.3)));
        bot = new Bot(hardwareMap, drive, Side.RED, telemetry);

        bot.initialize();
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + 4000) { //Scan for april tag for 4 seconds
            bot.updatePose();
            bot.aprilTagTelementary();
            telemetry.update();
        }

        Pose2d startPos = bot.botPose;
        telemetry.addData("Start Position: ", startPos);
        telemetry.update();
        bot.isOpModeRunning = true;

        Action RRPath = bot.drive.actionBuilder(startPos)
            .afterTime(0, bot.canon.setVelAction(launchSpeed))
            .strafeToLinearHeading(launchVec, toRadians(45))
            .afterTime(0, aimSequence(2.4, 0.7))
            .waitSeconds(2.4)


            .setTangent(toRadians(10))
            .splineToSplineHeading(new Pose2d(13, -40, toRadians(-90)), toRadians(-90))
            .strafeToConstantHeading(new Vector2d(13, -57.5))
            .waitSeconds(0.2)
            .setTangent(toRadians(90))
            .afterTime(0, bot.canon.setVelAction(launchSpeed))
            .splineToLinearHeading(launchPose, toRadians(180))
            //.afterTime(0, aimSequence())
            .waitSeconds(1.1)

            .setTangent(toRadians(-20))
            .splineToSplineHeading(new Pose2d(11, -50, toRadians(-125)), toRadians(-90))
            .strafeToConstantHeading(new Vector2d(12, -60))
            .waitSeconds(3.5)
            .setTangent(toRadians(90))
            .afterTime(0, bot.canon.setVelAction(launchSpeed))
            .splineToLinearHeading(launchPose, toRadians(180))
            //.afterTime(0, aimSequence())
            .waitSeconds(1.1)

            .build();

        Action RRPath2 = bot.drive.actionBuilder(startPos)
            .strafeToLinearHeading(launchVec, toRadians(45))
            .afterTime(0, aimSequence(2.3, 1.2))
            .waitSeconds(2.2)

            //.setTangent(toRadians(10))
            /*.splineTo(new Vector2d(13, -40), toRadians(-90))
            .strafeToConstantHeading(new Vector2d(13, -57.5))
            .waitSeconds(0.2)
            .setTangent(toRadians(90))
            .strafeToLinearHeading(launchVec, toRadians(45))

            .waitSeconds(1.2)*/

            .afterTime(0, bot.intake.setPower(-1.0))
            .setTangent(toRadians(0))
            .splineToSplineHeading(new Pose2d(PARAMS.xMiddle, PARAMS.yMiddle1, toRadians(-90)), toRadians(-90))
            .strafeToConstantHeading(new Vector2d(PARAMS.xMiddle, PARAMS.yMiddle2))
            .setTangent(toRadians(90))
            .splineToLinearHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(1.1)

            .afterTime(0, bot.intake.setPower(-1.0))
            .setTangent(toRadians(0))
            .splineToSplineHeading(new Pose2d(PARAMS.x1, PARAMS.y1, toRadians(PARAMS.angle)), toRadians(-90))
            .strafeToLinearHeading(new Vector2d(PARAMS.x1, PARAMS.y2), toRadians(PARAMS.angle))
            .waitSeconds(1.5)
            .setTangent(toRadians(90))
            .splineToLinearHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(1.1)

            .afterTime(0, bot.intake.setPower(-1.0))
            .setTangent(toRadians(0))
            .splineToSplineHeading(new Pose2d(PARAMS.x1, PARAMS.y1, toRadians(PARAMS.angle)), toRadians(-90))
            .strafeToLinearHeading(new Vector2d(PARAMS.x1, PARAMS.y2), toRadians(PARAMS.angle))
            .waitSeconds(1.5)
            .setTangent(toRadians(90))
            .splineToLinearHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(1.1)

            .afterTime(0, bot.intake.setPower(-1.0))
            .strafeToSplineHeading(new Vector2d(PARAMS.closeX, PARAMS.closeY1), toRadians(-90))
            //.strafeToConstantHeading(new Vector2d(PARAMS.closeX, PARAMS.closeY2))
            .lineToY(PARAMS.closeY2)
            .strafeToLinearHeading(launchVec, toRadians(45))
            .afterTime(0, aimSequence())
            .waitSeconds(1.1)

            /*.setTangent(toRadians(0))
            .splineToLinearHeading(new Pose2d(4.5, -62, toRadians(-125)), toRadians(-120))
            .waitSeconds(3.0)
            .strafeToLinearHeading(launchVec, toRadians(45))
            .afterTime(0, bot.intake.setPower(1.0))
            .waitSeconds(1.2)*/

            /*.strafeToLinearHeading(new Vector2d(9.7, -54), toRadians(-130))
            .strafeToLinearHeading(new Vector2d(9.7, -61), toRadians(-130))
            .waitSeconds(3.5)
            .setTangent(45)
            .strafeToLinearHeading(launchVec, toRadians(45))
            .afterTime(0, bot.intake.setPower(1.0))
            .waitSeconds(1.2)*/

            /*.setTangent(toRadians(0))
            .splineToLinearHeading(new Pose2d(4.5, -62, toRadians(-125)), toRadians(-120))
            .waitSeconds(3.0)
            .strafeToLinearHeading(launchVec, toRadians(45))
            .afterTime(0, bot.intake.setPower(1.0))
            .waitSeconds(1.2)*/

            /*.afterTime(0, bot.intake.setPower(-1.0))
            .strafeToLinearHeading(new Vector2d(9.7, -54), toRadians(-130))
            .strafeToLinearHeading(new Vector2d(9.7, -61), toRadians(-130))
            .waitSeconds(3.5)
            .strafeToLinearHeading(launchVec, toRadians(45))
            .afterTime(0, bot.intake.setPower(1.0))
            .waitSeconds(1.2)*/

            /*.turnTo(toRadians(-90))
            .strafeToConstantHeading(new Vector2d(-12.3, -53))
            .waitSeconds(0.2)
            .strafeToLinearHeading(launchVec, launchPose.heading.toDouble())
            .waitSeconds(1.2)*/
            .build();

        Action RRPath3 = new SequentialAction(
            bot.moveToImprecise(launchPose),
            new Wait(1.6),
            bot.moveToImprecise(new Pose2d(9.5, -61, toRadians(-125))),
            new Wait(1.6),
            bot.moveToImprecise(launchPose),
            new Wait(1.6),
            bot.moveToImprecise(new Pose2d(9.5, -61, toRadians(-125))),
            new Wait(1.6),
            bot.moveToImprecise(launchPose)
        );


        waitForStart();

        //END OF INIT


        bot.intake.setPowerManual(-1.0);
        Actions.runBlocking(new ParallelAction(
            RRPath2,
            new KeepRunning(bot.updatePoseAction()),
            new KeepRunning(bot.canon.cloneMotorPower()),
            new KeepRunning(bot.canon.setVelAction(launchSpeed))
        ));
    }

    public Action aimSequence() {
        return aimSequence(1.1, 0.0);
    }
    public Action aimSequence(double time1, double time2) {
        return new SequentialAction(
            bot.intake.setPower(0.0),
            bot.enableAprilTag(),
            bot.stopAction(),
            bot.gate.open(),
            new EndAfterFirstParallel(
                new Wait(time1),
                new ParallelAction(
                    new SequentialAction(
                        new EndAfterFirstParallel(
                            new Wait(0.6),
                            new KeepRunning(bot.aimAtGoal())
                        ),
                        bot.stopAction()
                    ),
                    new SequentialAction(
                        new Wait(time2),
                        bot.intake.setPower(-1.0)
                    )
                )
            ),
            bot.canon.setPowerAction(0),
            bot.gate.close(),
            bot.disableAprilTag()
        );
    }
}


