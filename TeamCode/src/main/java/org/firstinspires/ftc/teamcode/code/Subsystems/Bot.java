package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterFirstParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;
public class Bot {
    public Canon canon;
    public Gate gate;
    public Intake intake;

    public Bot(HardwareMap hardwareMap) {
        canon = new Canon(hardwareMap);
        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    public Action shootFar() {
        return new SequentialAction(
            gate.close(),
            new ParallelAction(
                canon.spinUp(canon.FAR_SPEED),
                intake.spinForDuration(0.5, 1.0)
            ),
            gate.open(),
            new Wait(0.3),
            new EndAfterFirstParallel(
                new SequentialAction(
                    intake.spinForDuration(0.9, -1.0),
                    new Wait(2.0),
                    intake.spinForDuration(0.5, -1.0),
                    new Wait(2.0),
                    intake.spinForDuration(1.0, -1.0),
                    new Wait(2.0)
                ),
                canon.maintainSpeed(canon.FAR_SPEED)
            ),
            canon.stopInstant(),
            gate.close()
        );
    }
}
