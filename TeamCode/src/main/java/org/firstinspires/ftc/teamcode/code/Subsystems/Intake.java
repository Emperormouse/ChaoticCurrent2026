package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
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
        public SpinForDuration(double time) {
            this.time = time*1000;
            start = System.currentTimeMillis();
        }
        public boolean run(TelemetryPacket t) {
            if (start + time > System.currentTimeMillis()) {
                intake();
                return true;
            } else {
                stop();
                return false;
            }
        }
    }
    public Action spinForDuration(double time) {
        return new SpinForDuration(time);
    }
}
