package org.firstinspires.ftc.teamcode.code.Auto;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
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
        public double x1 = 6.5;
        public double angle = -120;
        public double angleVel = -100;
        public double y1 = -50;
        public double y2 = -61.75;
        public double xMiddle = 10;
        public double yMiddle1 = -30;
        public double yMiddle2 = -61;
        public double closeX = -14;
        public double closeY1 = -33;
        public double closeY2 = -54.5;
        public double launchX = -15;
        public double launchY = -17;
        public double launchR = 40;
        public double lenBack = 10;
    }
    public static PARAMS PARAMS = new PARAMS();
    //hi

    MecanumDrive drive;
    Bot bot;
    Pose2d launchPose = new Pose2d(PARAMS.launchX, PARAMS.launchY, Math.toRadians(PARAMS.launchR));
    Vector2d launchVec = new Vector2d(launchPose.position.x, launchPose.position.y);
    double launchSpeed = 1400;

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-52.87, -36.2, Math.toRadians(-90)));
        bot = new Bot(hardwareMap, drive, Side.BLUE, telemetry);

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

        Action RRPath1 = bot.drive.actionBuilder(startPos)
            .strafeToLinearHeading(launchVec, toRadians(PARAMS.launchR))
            .afterTime(0, aimSequence(2.3, 1.1))
            .waitSeconds(2.3)
            .build();

        //END OF INIT
        waitForStart();

        Actions.runBlocking(bot.canon.setVelAction(launchSpeed));
        Actions.runBlocking(genPath(RRPath1));

        Action RRPath2 = bot.drive.actionBuilder(bot.botPose)
            .afterTime(0, bot.intake.setPower(-1.0))
            .setTangent(toRadians(0))
            .splineToSplineHeading(new Pose2d(PARAMS.xMiddle, PARAMS.yMiddle1, toRadians(-90)), toRadians(-90))
            .strafeToConstantHeading(new Vector2d(PARAMS.xMiddle, PARAMS.yMiddle2))
            .afterTime(0.3, bot.intake.setPower(0))
            .setTangent(toRadians(90))
            .splineToSplineHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(1.4)
            .build();

        Actions.runBlocking(genPath(RRPath2));

        Action RRPath3 = bot.drive.actionBuilder(bot.botPose)
            .afterTime(0, bot.intake.setPower(-1.0))
            .setTangent(toRadians(0))
            //.splineToSplineHeading(new Pose2d(PARAMS.x1, PARAMS.y1, toRadians(PARAMS.angle)), toRadians(-90))
            //.strafeToLinearHeading(new Vector2d(PARAMS.x1, PARAMS.y2), toRadians(PARAMS.angle))
            .splineToSplineHeading(new Pose2d(PARAMS.x1+PARAMS.lenBack*cos(toRadians(180+PARAMS.angleVel)), PARAMS.y2+PARAMS.lenBack*sin(toRadians(180+PARAMS.angleVel)), toRadians(PARAMS.angle)), toRadians(PARAMS.angleVel))
            .strafeToLinearHeading(new Vector2d(PARAMS.x1, PARAMS.y2), toRadians(PARAMS.angle))
            .waitSeconds(1.5)
            .afterTime(0.3, bot.intake.setPower(0))
            .setTangent(toRadians(90))
            .splineToSplineHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(1.4)
            .build();

        Actions.runBlocking(genPath(RRPath3));

        Action RRPath4 = bot.drive.actionBuilder(bot.botPose)
            .afterTime(0, bot.intake.setPower(-1.0))
            .setTangent(toRadians(0))
            //.splineToSplineHeading(new Pose2d(PARAMS.x1, PARAMS.y1, toRadians(PARAMS.angle)), toRadians(-90))
            //.strafeToLinearHeading(new Vector2d(PARAMS.x1, PARAMS.y2), toRadians(PARAMS.angle))
            .splineToSplineHeading(new Pose2d(PARAMS.x1+PARAMS.lenBack*cos(toRadians(180+PARAMS.angleVel)), PARAMS.y2+PARAMS.lenBack*sin(toRadians(180+PARAMS.angleVel)), toRadians(PARAMS.angle)), toRadians(PARAMS.angleVel))
            .strafeToLinearHeading(new Vector2d(PARAMS.x1, PARAMS.y2), toRadians(PARAMS.angle))
            .waitSeconds(1.5)
            .afterTime(0.3, bot.intake.setPower(0))
            .setTangent(toRadians(90))
            .splineToSplineHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(1.4)
            .build();

        Actions.runBlocking(genPath(RRPath4));

        Action RRPath5 = bot.drive.actionBuilder(bot.botPose)
            .afterTime(0, bot.intake.setPower(-1.0))
            .afterTime(0, bot.canon.setVelAction(1260))
            //.strafeToSplineHeading(new Vector2d(PARAMS.closeX, PARAMS.closeY1), toRadians(-90))
            //.strafeToConstantHeading(new Vector2d(PARAMS.closeX, PARAMS.closeY2))
            .setTangent(toRadians(-100))
            .splineToSplineHeading(new Pose2d(PARAMS.closeX, PARAMS.closeY1, toRadians(-90)), toRadians(-90))
            .strafeToConstantHeading(new Vector2d(PARAMS.closeX, PARAMS.closeY2))
            .afterTime(0.3, bot.intake.setPower(0))
            .strafeToSplineHeading(new Vector2d(-38.47, -15.2), toRadians(58))
            .afterTime(0, aimSequence())
            .waitSeconds(1.4)
            .build();

        Actions.runBlocking(genPath(RRPath5));

        /*Actions.runBlocking(
            new SequentialAction(
                bot.canon.setVelAction(launchSpeed),
                new ParallelAction(
                    RRPath,
                    new KeepRunning(bot.updatePoseAction()),
                    new KeepRunning(bot.canon.cloneMotorPower()),
                    new KeepRunning(bot.canon.setVelToTargetAction())
                ),
                bot.stopAction(),
                bot.intake.setPower(0),
                bot.canon.setPowerAction(0)
            )
        );*/
    }

    public Action genPath(Action path) {
        return new EndAfterFirstParallel(
            path,
            new ParallelAction(
                new KeepRunning(bot.updatePoseAction()),
                new KeepRunning(bot.canon.cloneMotorPower()),
                new KeepRunning(bot.canon.setVelToTargetAction())
            )
        );
    }

    public Action aimSequence() {
        return aimSequence(1.4, 0.3);
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
                            new Wait(1.1),
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
            bot.gate.close(),
            bot.disableAprilTag()
        );
    }
}


