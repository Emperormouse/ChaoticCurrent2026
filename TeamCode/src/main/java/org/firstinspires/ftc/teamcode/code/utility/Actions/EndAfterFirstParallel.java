package org.firstinspires.ftc.teamcode.code.utility.Actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class EndAfterFirstParallel implements Action {
    private Action a;
    private Action b;
    private boolean b_status = true;

    public EndAfterFirstParallel(Action a, Action b) {
        this.a = a;
        this.b = b;
    }

    public boolean run (TelemetryPacket t) {
        if (b_status) {
            b_status = b.run(t);
        }

        return a.run(t);
    }
}
