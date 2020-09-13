package org.firstinspires.ftc.teamcode.subsystems;

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

        left = hardwareMap.servo.get("intake_left");
        right = hardwareMap.servo.get("intake_right");
        flip = hardwareMap.servo.get("intake_flip");

    }

    public void run(boolean claw, boolean flip) {
        double clawPosition;

        if(claw && !this.claw) {
            clawPosition = CLAW_IN;
            this.claw = true;
        } else {
            clawPosition = CLAW_OUT;
        }
        if(!claw && this.claw) {
            this.claw = false;
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

        setClawPosition(clawPosition);

    }

    private void setClawPosition(double position) {
        left.setPosition(position);
        right.setPosition(position);
    }

    @Override
    public void stop() {

    }
}
