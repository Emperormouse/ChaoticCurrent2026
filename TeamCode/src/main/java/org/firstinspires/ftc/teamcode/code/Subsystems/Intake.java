package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    private DcMotorEx motor;
    private Bot bot;

    public Intake(HardwareMap hardwareMap, Bot bot) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.bot = bot;
    }

    public void intake() {
        motor.setPower(-1.0);
    }
    public void reverse() {
        motor.setPower(1.0);
    }
    public void stop() {
        motor.setPower(0.0);
    }

    public class SpinForDuration implements Action {
        private long start;
        private double time;
        private double speed;
        private boolean firstLoop = true;
        public SpinForDuration(double time, double speed) {
            this.time = time*1000;
            this.speed = speed;
        }
        public boolean run(TelemetryPacket t) {
            if (firstLoop) {
                start = System.currentTimeMillis();
                firstLoop = false;
            }
            if (start + time > System.currentTimeMillis()) {
                motor.setPower(speed);
                return true;
            } else {
                stop();
                return false;
            }
        }
    }

    public class IntakeWhenAtSpeed implements  Action {
        public boolean run(TelemetryPacket t) {
            if (Math.abs(bot.canon.motor.getVelocity()) + 40 >= Math.abs(bot.canon.CLOSE_SPEED)) {
                intake();
            } else {
                stop();
            }
            return true;
        }
    }

    public class IntakeUntilBallShot implements  Action {
        public boolean run(TelemetryPacket t) {
            if (Math.abs(bot.canon.motor.getVelocity()) + 40 >= Math.abs(bot.canon.CLOSE_SPEED)) {
                intake();
                return true;
            } else {
                stop();
                return false;
            }
        }
    }

    public Action spinForDuration(double time, double speed) {
        return new SpinForDuration(time, speed);
    }
    public Action intakeWhenAtSpeed() {
        return new IntakeWhenAtSpeed();
    }
    public Action intakeUntilBallShot() {
        return new IntakeUntilBallShot();
    }
}
