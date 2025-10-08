package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    public Canon canon;
    private Gate gate;
    private Intake intake;
    private final int SHOOT_SPEED_FAR = 1100;
    private final int SHOOT_SPEED_CLOSE = 900;
    private final double TIME_TO_SHOOT = 4.0;

    public Outtake(HardwareMap hardwareMap) {
        canon = new Canon(hardwareMap);
        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap);
    }

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
    public Action shootFar() {
        return new SequentialAction(
            canon.spinUp(SHOOT_SPEED_FAR),
            new ParallelAction(
                canon.maintainSpeed(SHOOT_SPEED_FAR),
                new SequentialAction(
                    new ParallelAction(
                        new Wait(2.0),
                        intake.spinForDuration(0.5)
                    ),
                    new ParallelAction(
                        new Wait(2.0),
                        intake.spinForDuration(0.5)
                    ),
                    new ParallelAction(
                        new Wait(2.0),
                        intake.spinForDuration(0.5)
                    )
                )
            )
        );
    }
}
