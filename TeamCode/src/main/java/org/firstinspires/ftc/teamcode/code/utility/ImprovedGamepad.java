package org.firstinspires.ftc.teamcode.code.utility;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.code.utility.Button;

import java.lang.reflect.Field;

public class ImprovedGamepad {
    public Button a;
    public Button b;
    public Button x;
    public Button y;

    public Button dpad_down;
    public Button dpad_up;
    public Button dpad_right;
    public Button dpad_left;

    public Button right_bumper;
    public Button left_bumper;

    public Button start;
    public Button options;

    public Analogue right_trigger;
    public Analogue left_trigger;

    public Gamepad pad; //Reference to regular gamepad

    public ImprovedGamepad(Gamepad pad) {
        a = new Button(() -> pad.a);
        b = new Button(() -> pad.b);
        x = new Button(() -> pad.x);
        y = new Button(() -> pad.y);

        dpad_down = new Button(() -> pad.dpad_down);
        dpad_up = new Button(() -> pad.dpad_up);
        dpad_right = new Button(() -> pad.dpad_right);
        dpad_left = new Button(() -> pad.dpad_left);

        right_bumper = new Button(() -> pad.right_bumper);
        left_bumper = new Button(() -> pad.left_bumper);

        start = new Button(() -> pad.start);
        options = new Button(() -> pad.options);

        right_trigger = new Analogue(() -> pad.right_trigger, () -> pad.right_trigger > 0.05);
        left_trigger = new Analogue(() -> pad.left_trigger, () -> pad.left_trigger > 0.05);

        this.pad = pad;

        update();
    }

    //Loop through each button and update it
    public void update() {
        for (Field f : this.getClass().getDeclaredFields()) {
            try {
                Object obj = f.get(this);
                if (obj instanceof Button) {
                    ((Button)obj).update();
                }
            } catch(Exception e) {continue;}
        }
    }
}