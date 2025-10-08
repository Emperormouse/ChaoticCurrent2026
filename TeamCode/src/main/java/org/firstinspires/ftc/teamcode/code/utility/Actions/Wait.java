package org.firstinspires.ftc.teamcode.code.utility.Actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class Wait implements Action {
    private long start;
    private double time;
    public Wait(double time) {
        this.time = time*1000;
        start = System.currentTimeMillis();
    }
    public boolean run(TelemetryPacket t) {
        return (start + time > System.currentTimeMillis());
    }
}
