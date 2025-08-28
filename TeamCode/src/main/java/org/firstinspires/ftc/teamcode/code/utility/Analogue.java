package org.firstinspires.ftc.teamcode.code.utility;

public class Analogue extends Button {
    private final AnalogueStatus aStatus;
    public double val;

    public Analogue(AnalogueStatus aStatus, Button.ButtonStatus bStatus) {
        super(bStatus);
        this.aStatus = aStatus;
    }

    public void update() {
        super.update();
        val = aStatus.poll();
    }

    @FunctionalInterface
    public interface AnalogueStatus {
        double poll();
    }
}
