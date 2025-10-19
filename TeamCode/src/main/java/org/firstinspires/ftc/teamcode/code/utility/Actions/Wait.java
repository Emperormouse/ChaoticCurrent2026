package org.firstinspires.ftc.teamcode.code.utility.Actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class Wait implements Action {
    private long start;
    private double time;
    private boolean firstLoop = true;
    public Wait(double time) {
        this.time = time*1000;
    }
    public boolean run(TelemetryPacket t) {
        if (firstLoop) {
            start = System.currentTimeMillis();
            firstLoop = false;
        }
        return (start + time > System.currentTimeMillis());
    }
}
