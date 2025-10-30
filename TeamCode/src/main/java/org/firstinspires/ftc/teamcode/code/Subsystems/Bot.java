package org.firstinspires.ftc.teamcode.code.Subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterFirstParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.KeepRunning;
import org.firstinspires.ftc.teamcode.code.utility.Actions.PIDController;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;
public class Bot {
    public Canon canon;
    public Gate gate;
    public Intake intake;
    public DistanceSensor distanceSensor;
    public Localizer localizer;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    public Bot(HardwareMap hardwareMap, Localizer localizer) {
        canon = new Canon(hardwareMap);
        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap, this);
        this.localizer = localizer;

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
    }

    public Action shootClose() {
        return new SequentialAction(
            canon.spinUp(canon.CLOSE_SPEED),
            gate.open(),
            new Wait(0.3),
            new EndAfterFirstParallel(
                new Wait(7),
                intake.intakeWhenAtSpeed()
            ),
            canon.setPowerInstant(0),
            gate.close()
        );
    }

    public void moveFieldCentric(double x, double y, double r) {
        moveFieldCentric(x, y, r, 1.0);
    }

    public void moveFieldCentric(double x, double y, double r, double speed) {
        double frPower = 0;
        double flPower = 0;
        double brPower = 0;
        double blPower = 0;

        double botRot = localizer.getPose().heading.toDouble() + Math.toRadians(90);

        frPower += r;
        brPower += r;
        flPower -= r;
        blPower -= r;

        //FORWARD-DIRECTION
        frPower += y * cos(botRot);
        brPower += y * cos(botRot);
        flPower += y * cos(botRot);
        blPower += y * cos(botRot);

        frPower -= y * sin(botRot);
        brPower += y * sin(botRot);
        flPower += y * sin(botRot);
        blPower -= y * sin(botRot);

        //SIDEWAYS-DIRECTION
        frPower += x * sin(botRot);
        brPower += x * sin(botRot);
        flPower += x * sin(botRot);
        blPower += x * sin(botRot);

        frPower += x * cos(botRot);
        brPower -= x * cos(botRot);
        flPower -= x * cos(botRot);
        blPower += x * cos(botRot);

        double denominator = max(1, max(max(max(abs(frPower), abs(brPower)), abs(flPower)), abs(blPower)));
        denominator /= speed;

        frontLeft.setPower(flPower / denominator);
        frontRight.setPower(frPower / denominator);
        backLeft.setPower(blPower / denominator);
        backRight.setPower(brPower / denominator);
    }

    //drives to location
    public class MoveTo implements Action {
        private final double pRotational = 1.3;
        private final double pX = 0.08;
        private final double pY = 0.05;
        private Pose2d targetPose;
        //private PIDController pidX;

        public MoveTo(Pose2d targetPose) {
            this.targetPose = targetPose;
            //pidX = new PIDController(pTranslational, 0, 0);
        }

        public boolean run(TelemetryPacket t) {
            Pose2d currentPose = localizer.getPose();
            double diffX = targetPose.position.x - currentPose.position.x;
            double diffY = targetPose.position.y - currentPose.position.y;
            double diffR = targetPose.heading.toDouble() - currentPose.heading.toDouble();

            moveFieldCentric(diffX*pX, -diffY*pY, diffR*pRotational, 0.8);

            if (Math.abs(diffX)>1.3 || Math.abs(diffY)>1.3 || Math.abs(diffR)>Math.toRadians(1.2)) {
                return true;
            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                return false;
            }
        }
    }

    public Action moveTo(Pose2d targetPose) {
        return new MoveTo(targetPose);
    }
    public Action moveTo(Pose2d targetPose, double time) {
        return new SequentialAction(
            new MoveTo(targetPose),
            settle(targetPose, time)
        );
    }

    //infinitely settles
    public Action settle(Pose2d targetPose) {
        return new KeepRunning(new MoveTo(targetPose));
    }
    //settles for amount of time in seconds
    public Action settle(Pose2d targetPose, double time) {
        return new EndAfterFirstParallel(
            new Wait(time),
            settle(targetPose)
        );
    }

}
