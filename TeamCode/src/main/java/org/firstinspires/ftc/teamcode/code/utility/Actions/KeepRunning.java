package org.firstinspires.ftc.teamcode.code.utility.Actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class KeepRunning implements Action {
    Action a;
    public KeepRunning(Action a) {
        this.a = a;
    }

    public boolean run(TelemetryPacket t) {
        a.run(t);
        return true;
    }
}
