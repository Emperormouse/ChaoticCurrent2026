package org.firstinspires.ftc.teamcode.code.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    public Canon canon;
    private Servo barrier;
    private final int SHOOT_SPEED_FAR = 2200;
    private final int SHOOT_SPEED_CLOSE = 1000;
    private final double TIME_TO_SHOOT = 4.0;

    public Outtake(HardwareMap hardwareMap) {
        canon = new Canon(hardwareMap);
    }

    public class Shoot implements Action {
        private Action canonShoot;
        private long startTime;
        public Shoot(int speed) {
            canonShoot = canon.maintainSpeed(speed);
            startTime = System.currentTimeMillis();
        }

        public boolean run(TelemetryPacket t) {
            canonShoot.run(t);
            return (System.currentTimeMillis() - startTime < (1000 * TIME_TO_SHOOT));
        }
    }
    public Action shootFar() {
        return new Shoot(SHOOT_SPEED_FAR);
    }
    public Action shootClose() {
        return new Shoot(SHOOT_SPEED_CLOSE);
    }
}
