package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public class WobbleGrabber extends Subsystem {
    public Servo wobbleTurn;
    private boolean wobbleButton;
    private double wobbleOn = 1.0;
    @Override
    public void init() {

    }
    public void run(boolean wobbleButton) {
        if(wobbleButton && !this.wobbleButton){
            wobbleOn *= -1;
            this.wobbleButton = true;
        }
        if(!wobbleButton && this.wobbleButton){
            wobbleButton = false;
        }
        if(wobbleOn > 0){
            wobbleTurn.setPosition(1);
        }
        else if(wobbleOn < 0){
        }
    }

    @Override
    public void stop() {

    }
}
