package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Set;

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
    public void setPowerManual(double pow) {
        motor.setPower(pow);
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
        private long start;
        private boolean firstTime = true;
        //15 length
        private boolean[] arr = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
        private long lastTime = 0;

        public boolean run(TelemetryPacket t) {
            boolean isAtSpeed = Math.abs(bot.canon.motor.getVelocity() - bot.canon.targetVel) <= 40;

            if (System.currentTimeMillis() > lastTime + 20) {
                lastTime = System.currentTimeMillis();
                for (int i = arr.length-1; i >= 1; i--) {
                    arr[i] = arr[i-1];
                }
                arr[0] = isAtSpeed;
            }

            if (firstTime) {
                firstTime = false;
                start = System.currentTimeMillis();
            }

            boolean lastFiveChecksWereTrue = true;
            for (boolean b : arr) {
                lastFiveChecksWereTrue &= b;
            }
            if (lastFiveChecksWereTrue) {
                intake();
            } else {
                stop();
            }
            return true;
        }
    }


    public class IntakeUntilBallShot implements  Action {
        public boolean run(TelemetryPacket t) {
            if (Math.abs(bot.canon.motor.getVelocity()) + 30 >= Math.abs(bot.canon.CLOSE_SPEED)) {
                //intake();
                motor.setPower(-1.0);
                return true;
            } else {
                stop();
                return false;
            }
        }
    }

    public class IntakeUntilBallShotLAST implements  Action {
        public boolean run(TelemetryPacket t) {
            if (Math.abs(bot.canon.motor.getVelocity()) + 30 >= Math.abs(bot.canon.CLOSE_SPEED_LAST)) {
                intake();
                return true;
            } else {
                stop();
                return false;
            }
        }
    }

    public class SetPowerAction implements Action {
        private double pow;
        public SetPowerAction(double p) {
            pow = p;
        }
        public boolean run(TelemetryPacket t) {
            setPowerManual(pow);
            return false;
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
    public Action intakeUntilBallShotLAST() {
        return new IntakeUntilBallShotLAST();
    }
    public Action setPower(double pow) {
        return new SetPowerAction(pow);
    }
}
