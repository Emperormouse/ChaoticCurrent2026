package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterFirstParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;
public class Bot {
    public Canon canon;
    public Gate gate;
    public Intake intake;
    public DistanceSensor distanceSensor;

    public Bot(HardwareMap hardwareMap) {
        canon = new Canon(hardwareMap);
        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap, this);
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
    }

    public Action shootClose() {
        return new SequentialAction(
            gate.open(),
            new Wait(0.3),
            new EndAfterFirstParallel(
                new SequentialAction(
                    intake.intakeUntilBallShot(),
                    new Wait(2.0),
                    intake.intakeUntilBallShot(),
                    new Wait(2.0),
                    intake.intakeUntilBallShot()
                ),
                canon.maintainSpeed(canon.CLOSE_SPEED)
            ),
            canon.stopInstant(),
            gate.close()
        );
    }
}
