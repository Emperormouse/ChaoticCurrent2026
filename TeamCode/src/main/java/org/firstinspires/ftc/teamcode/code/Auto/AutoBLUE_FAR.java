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
public class AutoBLUE_FAR extends LinearOpMode {
    public static class PARAMS {
        public double launchX = 52;
        public double launchY = -15;
        public double launchR = 28;
        public double cycleX = 63.5;
        public double cycleY = -64;
        public double yRow1 = -30;
        public double yRow2 = -60;
        public double xRow = 30;

    }
    public static PARAMS PARAMS = new PARAMS();

    MecanumDrive drive;
    Bot bot;

    Pose2d launchPoseFar = new Pose2d(PARAMS.launchX, PARAMS.launchY, Math.toRadians(PARAMS.launchR));
    Vector2d launchVecFar = new Vector2d(launchPoseFar.position.x, launchPoseFar.position.y);
    double launchSpeed = 1740;

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


        Action RRPath = bot.drive.actionBuilder(startPos)
            .strafeToLinearHeading(launchVecFar, toRadians(20))
            .waitSeconds(2.0)
            .afterTime(0, aimSequence(4.4, 1.0))
            .waitSeconds(4.5)

            .afterTime(0, bot.intake.setPower(-1.0))
            .setTangent(toRadians(180))
            .splineToSplineHeading(new Pose2d(PARAMS.xRow, PARAMS.yRow1, toRadians(-90)), toRadians(-90))
            .strafeToConstantHeading(new Vector2d(PARAMS.xRow, PARAMS.yRow2))
            .waitSeconds(0.5)
            .afterTime(0.5, bot.intake.setPower(0))
            .strafeToLinearHeading(launchVecFar, toRadians(20))
            .afterTime(0, aimSequence())
            .waitSeconds(5.0)

            .turnTo(toRadians(-90))
            .afterTime(0, bot.intake.setPower(-1.0))
            .strafeToConstantHeading(new Vector2d(PARAMS.cycleX, PARAMS.cycleY))
            .waitSeconds(1.0)
            .strafeToSplineHeading(launchVecFar, toRadians(20))
            /*.strafeToConstantHeading(launchVecFar)
            .turnTo(toRadians(20))*/
            .afterTime(0, aimSequence())
            .waitSeconds(5.0)

            .strafeToConstantHeading(new Vector2d(PARAMS.cycleX, PARAMS.cycleY))
            .waitSeconds(1.0)

            .build();



        waitForStart();

        //END OF INIT


        Actions.runBlocking(
            new ParallelAction(
                RRPath,
                new KeepRunning(bot.updatePoseAction()),
                new KeepRunning(bot.canon.cloneMotorPower()),
                new KeepRunning(bot.canon.setVelAction(launchSpeed))
            )
        );
    }

    public Action aimSequence() {
        return aimSequence(4.9, 1.0);
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
                            new Wait(3.0),
                            new KeepRunning(bot.aimAtGoal())
                        ),
                        bot.stopAction()
                    ),
                    new SequentialAction(
                        new Wait(time2),
                        new KeepRunning(bot.intake.intakeWhenAtSpeed())
                    )
                )
            ),
            bot.canon.setPowerAction(0),
            bot.gate.close(),
            bot.disableAprilTag()
        );
    }
}


