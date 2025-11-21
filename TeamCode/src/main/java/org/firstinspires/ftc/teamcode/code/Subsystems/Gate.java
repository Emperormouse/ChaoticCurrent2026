package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
    private Servo left;
    private Servo right;

    public Gate(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "axon_gate");
        right = hardwareMap.get(Servo.class, "axon_gate2");
        closeManual();
    }

    public void openManual() {
        left.setPosition(0.3);
        right.setPosition(0.05);
    }
    public void closeManual() {
        left.setPosition(0.05);
        right.setPosition(0.25);
    }
    public void holdManual() {
        left.setPosition(left.getPosition());
        right.setPosition(right.getPosition());
    }

    public class Open implements Action {
        public boolean run(TelemetryPacket t) {
            openManual();
            return false;
        }
    }
    public class Close implements Action {
        public boolean run(TelemetryPacket t) {
            closeManual();
            return false;
        }
    }
    public class Hold implements Action {
        public boolean run(TelemetryPacket t) {
            holdManual();
            return false;
        }
    }

    public Action open() {
        return new Open();
    }
    public Action close() {
        return new Close();
    }
    public Action hold() {
        return new Hold();
    }
}
//0 1
//0.3 0.7