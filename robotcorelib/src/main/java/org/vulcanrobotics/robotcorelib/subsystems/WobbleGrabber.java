package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public class WobbleGrabber extends Subsystem {
    public Servo wobbleTurn;
    public Servo wobbleGrab;
    private boolean wobbleTurnButton;
    public boolean wobbleGrabButton;
    private double wobbleTurnOn = -1.0;
    private double wobbleGrabOn = 1.0;

    @Override
    public void init() {

    }

    public void run(boolean wobbleTurnButton, boolean wobbleGrabButton) {
        if (wobbleTurnButton && !this.wobbleTurnButton) {
            wobbleTurnOn *= -1;
            this.wobbleTurnButton = true;
        }
        if (!wobbleTurnButton && this.wobbleTurnButton) {
            wobbleTurnButton = false;
        }
        if (wobbleTurnOn > 0) {
            wobbleTurn.setPosition(1);
        }
        else if (wobbleTurnOn < 0) {
            wobbleTurn.setPosition(0);
        }


        if (wobbleGrabButton && !this.wobbleGrabButton) {
            wobbleGrabOn *= -1;
            this.wobbleTurnButton = true;
        }
        if (!wobbleGrabButton && this.wobbleGrabButton) {
            wobbleGrabButton = false;
        }
        if (wobbleGrabOn > 0) {
            wobbleGrab.setPosition(1);
        }
       else if (wobbleGrabOn < 0) {
            wobbleGrab.setPosition(0);
        }



    }
        @Override
        public void stop() {}
}