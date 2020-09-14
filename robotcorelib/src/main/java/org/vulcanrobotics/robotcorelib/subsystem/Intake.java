package org.vulcanrobotics.robotcorelib.subsystem;

import com.qualcomm.robotcore.hardware.Servo;

import static org.vulcanrobotics.robotcorelib.framework.Constants.*;
import org.vulcanrobotics.robotcorelib.subsystem.Subsystem;

public class Intake extends Subsystem {

    private Servo left;
    private Servo right;
    private Servo flip;

    private boolean claw;
    private boolean flipped;
    private int flipPos = 1;

    @Override
    public void init() {

        left = hardwareMap.servo.get("claw_left");
        right = hardwareMap.servo.get("claw_right");
        flip = hardwareMap.servo.get("claw_flip");

    }

    public void run(boolean claw, boolean flip) {
        double clawPosition;

        if(claw) {
            clawPosition = CLAW_IN;
        } else {
            if(flipPos > 0) {

            }
            clawPosition = CLAW_OUT;
        }

        if(flip && !flipped) {
            flipPos *= -1;
            flipped = true;
        }
        if(!flip && flipped) {
            flipped = false;
        }
        if(flipPos < 0) {
            this.flip.setPosition(FLIP_IN);
        } else {
            this.flip.setPosition(FLIP_OUT);
        }

//        if(flip) {
//            this.flip.setPosition(FLIP_OUT);
//        } else {
//            this.flip.setPosition(FLIP_IN);
//        }

        setClawPosition(clawPosition);

    }

    private void setClawPosition(double position) {
        left.setPosition(position - 0.1);
        if(flipPos > 0) {
            right.setPosition(1 - 0.8);

        } else {
            right.setPosition(1 - position);
        }
    }

    @Override
    public void stop() {

    }
}
