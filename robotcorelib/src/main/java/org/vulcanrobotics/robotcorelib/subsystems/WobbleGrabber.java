package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public class WobbleGrabber extends Subsystem {
    public Servo wobbleTurn;
    public Servo wobbleGrab;
    public DcMotor wobbleLift;
    public double limitRange = 200;
    public double limitMin = 0;
    public double limitMax = 2000;
    private boolean wobbleTurnButton;
    private boolean wobbleGrabButton;
    private double wobblePower = 1;
    private double wobbleLiftJoystick = -1.0;

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


       telemetry.addData("wobble_height", wobbleLift.getCurrentPosition());
       if(wobbleLiftJoystick < 0){
           if(wobbleLift.getCurrentPosition() >= limitMax) {
           }
           wobbleLift.setPower(wobblePower);
       }
       if(wobbleLiftJoystick > 0){
           wobbleLift.setPower(-wobblePower);
       }
       else{
           wobbleLift.setPower(0);
       }

    }
        @Override
        public void stop() {}
}