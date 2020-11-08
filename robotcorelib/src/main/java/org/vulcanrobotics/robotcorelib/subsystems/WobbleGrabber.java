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
    public double limitRange = 400;
    public double limitMin = 0;
    public double limitMax = 1750;
    private boolean wobbleTurnButton;
    private boolean wobbleGrabButton;
    private double liftPower = 1;
    private double wobbleLiftJoystick = -1.0;

    private double wobbleTurnOn = -1.0;
    private double wobbleGrabOn = -1.0;

    @Override
    public void init() {

        wobbleTurn = hardwareMap.servo.get("wobble_turn");
        wobbleGrab = hardwareMap.servo.get("wobble_grab");
        wobbleLift = hardwareMap.dcMotor.get("wobble_lift");

    }

    public void run(boolean wobbleTurnButton, boolean wobbleGrabButton, double wobbleLiftJoystick) {

        if (wobbleTurnButton && !this.wobbleTurnButton) {
            wobbleTurnOn *= -1;
            this.wobbleTurnButton = true;
        }
        if (!wobbleTurnButton && this.wobbleTurnButton) {
            this.wobbleTurnButton = false;
        }
        if (wobbleTurnOn > 0) {
            wobbleTurn.setPosition(.70);
        }
        else  {
            wobbleTurn.setPosition(0);
        }


        if (wobbleGrabButton && !this.wobbleGrabButton) {
            wobbleGrabOn *= -1;
            this.wobbleTurnButton = true;
        }
        if (!wobbleTurnButton && this.wobbleTurnButton) {
            this.wobbleTurnButton = false;
        }
        if (wobbleGrabOn > 0) {
            wobbleGrab.setPosition(.70);
        }
        else  {
            wobbleGrab.setPosition(0);
        }


       telemetry.addData("wobble_height", wobbleLift.getCurrentPosition());
           if(wobbleLiftJoystick > 0 && wobbleLift.getCurrentPosition() >= limitMax - limitRange) {
               liftPower = (limitMax - wobbleLift.getCurrentPosition())/limitRange;
           }

           if(wobbleLiftJoystick < 0 && wobbleLift.getCurrentPosition() <= limitMin + limitRange) {
               liftPower = -((wobbleLift.getCurrentPosition() - limitMin)/limitRange);
           }

       else{
            liftPower = wobbleLiftJoystick;
       }
        wobbleLift.setPower(liftPower);
    }
        @Override
        public void stop() {}
}