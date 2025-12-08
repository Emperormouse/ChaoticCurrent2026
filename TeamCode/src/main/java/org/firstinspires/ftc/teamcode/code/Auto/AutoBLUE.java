package org.firstinspires.ftc.teamcode.code.Auto;

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
import org.firstinspires.ftc.teamcode.code.Subsystems.Bot;
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterEitherParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterFirstParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.KeepRunning;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;
import org.firstinspires.ftc.teamcode.code.utility.Op;
import org.firstinspires.ftc.teamcode.code.utility.Side;

@Autonomous
public class AutoBLUE extends LinearOpMode {
    MecanumDrive drive;
    Bot bot;
    Pose2d launchPose = new Pose2d(-22, -17, Math.toRadians(50));

    public void waitSeconds(double time) {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + time*1000);
    }

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-49.5, -50, Math.toRadians(56)));
        bot = new Bot(hardwareMap, drive, Side.BLUE, telemetry);

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

        Action path = new SequentialAction(
            //SHOOT FIRST 3 BALLS

            shootSequence(),
            bot.canon.setPowerAction(0),

            //GRAB SECOND 3 BALLS
            bot.intake.setPower(-1.0),
            bot.moveToImprecise(new Pose2d(-13, -12, toRadians(-90))),
            new EndAfterEitherParallel(
                new Wait(2.3),
                bot.moveToImprecise(new Pose2d(-13, -53, toRadians(-90)), 1.0)
            ),
            new Wait(0.6),
            bot.intake.setPower(0),

            bot.moveRelativeAction(0, -1.0, 0, 1.0),
            new Wait(0.3),

            //SHOOT SECOND 3 BALLS
            /*bot.intake.setPower(1),
            new Wait(0.5),
            bot.intake.setPower(0),*/

            shootSequence(),
            bot.canon.setPowerAction(0),

            //GRAB THIRD 3 BALLS
            bot.intake.setPower(-1.0),
            bot.moveToImprecise(new Pose2d(6, -6, toRadians(-80))),
            new EndAfterEitherParallel(
                new Wait(2.3),
                bot.moveToImprecise(new Pose2d(6, -56.5, toRadians(-80)), 1.0)
            ),
            new Wait(0.5),
            bot.intake.setPower(0),

            bot.moveRelativeAction(0, -1.0, 0, 1.0),
            new Wait(0.6),

            //SHOOT THIRD 3 BALLS
            /*bot.intake.setPower(1),
            new Wait(0.5),
            bot.intake.setPower(0),*/
            shootSequence(),
            bot.canon.setPowerAction(0)
        );

        waitForStart();

        //END OF INIT

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                    new EndAfterEitherParallel(
                        path,
                        new Wait(29.3)
                    ),
                    bot.moveRelativeAction(-1.0, 0, 0, 0.7)
                ),

                bot.updatePoseUsingAprilTagAction()
            )
        );
    }

    public Action shootSequence() {
        return new SequentialAction(
            bot.moveToImprecise(launchPose),

            new EndAfterFirstParallel(
                bot.shootClose(Op.AUTO),
                new SequentialAction(
                    new EndAfterFirstParallel(
                        new Wait(1.0),
                        new KeepRunning(bot.moveToLaunchArc())
                    ),
                    bot.stopAction()
                )
            ),
            bot.gate.close()
        );

    }
}













/* package org.firstinspires.ftc.teamcode.code.Auto;

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
        bot = new Bot(hardwareMap, drive, Side.BLUE, telemetry);

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

        Action path = new SequentialAction(
            //SHOOT FIRST 3 BALLS
            new EndAfterEitherParallel(
                new Wait(3.0),
                bot.moveToLaunchArc()
            ),
            new Wait(0.5),
            bot.shootClose(Op.AUTO),
            bot.canon.setPowerAction(0),

            //GRAB SECOND 3 BALLS
            bot.intake.setPower(-1.0),
            bot.moveToImprecise(new Pose2d(-11, -12, toRadians(-90))),
            new EndAfterEitherParallel(
                new Wait(2.3),
                bot.moveToImprecise(new Pose2d(-11, -53, toRadians(-90)), 1.0)
            ),
            new Wait(0.6),
            bot.intake.setPower(0),

            //SHOOT SECOND 3 BALLS
            new EndAfterEitherParallel(
                new Wait(4.5),
                bot.moveToLaunchArc()
            ),
            new Wait(0.5),
            bot.shootClose(Op.AUTO),
            bot.canon.setPowerAction(0),

            //GRAB THIRD 3 BALLS
            bot.intake.setPower(-1.0),
            bot.moveToImprecise(new Pose2d(32.5, -6, toRadians(-85))),
            new EndAfterEitherParallel(
                new Wait(2.3),
                bot.moveToImprecise(new Pose2d(32.5, -57.25, toRadians(-85)), 1.0)
            ),
            new Wait(0.5),
            bot.intake.setPower(0),

            //SHOOT THIRD 3 BALLS
            new EndAfterEitherParallel(
                new Wait(5),
                bot.moveToLaunchArc()
            ),
            new Wait(0.5),
            bot.shootClose(Op.AUTO),
            bot.canon.setPowerAction(0)
        );

        waitForStart();

        //END OF INIT

        Actions.runBlocking(new ParallelAction(
                path,
                bot.updatePoseUsingAprilTagAction()
            )
        );
    }
}
*/


