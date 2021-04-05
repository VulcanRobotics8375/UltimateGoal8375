package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public class WobbleGrabber extends Subsystem {
    public Servo wobbleTurn;
    public Servo wobbleGrab;
    private double wobbleMode = -1.0;
    private double limitRange = 500;
    private double limitMin = 0;
    private double limitMax = 1750;
    private boolean wobbleTurnButton = false;
    private boolean wobbleGrabButton = false;

    private double wobbleTurnOn = -1.0;

    private double wobbleGrabOn = -1.0;

    @Override
    public void init() {

        wobbleTurn = hardwareMap.servo.get("wobble_turn");
        wobbleGrab = hardwareMap.servo.get("wobble_grab");

    }

    /**
     * add any auto init stuff to this, rn its just run to position initialization
     * runs alongside init(), not instead of
     */
    /*
    public void autoInit() {
        wobbleLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

     */

    public void run(boolean wobbleTurnButton, boolean wobbleGrabButton, double wobbleLiftJoystick) {

        if (wobbleTurnButton && !this.wobbleTurnButton) {
            wobbleTurnOn *= -1;
            if(wobbleTurnOn < 0) {
                if(wobbleGrabOn < 0) {
                    wobbleGrabOn *= -1;
                }
            }
            this.wobbleTurnButton = true;
        }
        if (!wobbleTurnButton && this.wobbleTurnButton) {
            this.wobbleTurnButton = false;
        }
        if (wobbleTurnOn > 0) {
            wobbleTurn.setPosition(0.02);
        }
        if (wobbleTurnOn < 0)  {

            wobbleTurn.setPosition(0.62);
        }


        if (wobbleGrabButton && !this.wobbleGrabButton) {
            wobbleGrabOn *= -1;

            this.wobbleGrabButton = true;
        }
        if (!wobbleGrabButton && this.wobbleGrabButton) {
            this.wobbleGrabButton = false;
        }
        if(wobbleGrabOn > 0) {
          wobbleGrab.setPosition(.05);

        } if(wobbleGrabOn < 0) {
            wobbleGrab.setPosition(1.5);
        }
        /*
       telemetry.addData("wobble height", wobbleLift.getCurrentPosition());
       if(wobbleLiftJoystick > 0 && wobbleLift.getCurrentPosition() >= limitMax - limitRange) {
           liftPower = ((limitMax - wobbleLift.getCurrentPosition())/limitRange) * wobbleLiftJoystick;
       }

       else if(wobbleLiftJoystick < 0 && wobbleLift.getCurrentPosition() <= limitMin + limitRange) {
           liftPower = ((wobbleLift.getCurrentPosition() - limitMin)/limitRange) * wobbleLiftJoystick;
       } else {
            liftPower = wobbleLiftJoystick;
       }
        wobbleLift.setPower(liftPower);

         */
    }

    public void setGrabPosition(double position) {
        wobbleGrab.setPosition(position);
    }

    public void setTurnPosition(double position) {
        wobbleTurn.setPosition(position);
    }
/*
    private boolean liftRunningAuto = false;
    public void moveLiftToPosition(int position, double power) {
        if(wobbleLift.isBusy()) {
            liftRunningAuto = false;
        }
        if(!liftRunningAuto) {
            wobbleLift.setTargetPosition(position);
            liftRunningAuto = true;
        }
        wobbleLift.setPower(power);



    }

 */

        @Override
        public void stop() {}
}