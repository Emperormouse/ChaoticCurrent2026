package org.firstinspires.ftc.teamcode.code.utility.Actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class EndAfterEitherParallel implements Action {
    private Action a;
    private Action b;

    public EndAfterEitherParallel(Action a, Action b) {
        this.a = a;
        this.b = b;
    }

    public boolean run (TelemetryPacket t) {
        return a.run(t) & b.run(t);
    }
}
