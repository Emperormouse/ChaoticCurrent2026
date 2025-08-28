package org.firstinspires.ftc.teamcode.examples.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo servo;
    public Claw(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "servo1");
        open();
    }

    public void close() {
        servo.setPosition(0.4);
    }
    public void open() {
        servo.setPosition(0.6);
    }
}
