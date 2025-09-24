package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void startIntake() {
        motor.setPower(1.0);
    }
    public void reverse() {
        motor.setPower(-1.0);
    }
    public void stop() {
        motor.setPower(0.0);
    }
}
