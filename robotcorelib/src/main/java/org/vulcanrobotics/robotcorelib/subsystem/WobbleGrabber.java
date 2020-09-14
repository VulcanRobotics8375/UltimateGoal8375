package org.vulcanrobotics.robotcorelib.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.robot.Robot;
import org.vulcanrobotics.robotcorelib.subsystem.Subsystem;

public class WobbleGrabber extends Subsystem {

    private DcMotor lift;
    private Servo flip, claw;

    private int clawOn = 1;
    private boolean clawPressed;
    private boolean flipPressed;
    private int flipOn = 1;

    @Override
    public void init() {

        lift = hardwareMap.dcMotor.get("wobble_lift");
        flip = hardwareMap.servo.get("wobble_flip");
        claw = hardwareMap.servo.get("wobble_claw");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void run(double liftPower, boolean flip, boolean claw) {
        double power;
        double liftHighLimit = Constants.LIFT_HIGH - Constants.LIMIT_RANGE;

        double clawPos;

        if(claw && !clawPressed) {
            clawOn *= -1;
            clawPressed = true;
        }
        if(!claw && clawPressed) {
            clawPressed = false;
        }

        if(clawOn < 0) {
            clawPos = Constants.WOBBLE_CLAW_IN;
        } else {
            clawPos = Constants.WOBBLE_CLAW_OUT;
        }

        if(flip && !flipPressed) {
            flipOn *= -1;
            flipPressed = true;
        }
        if(!flip && flipPressed) {
            flipPressed = false;
        }

        if(flipOn < 0) {
            this.flip.setPosition(Constants.WOBBLE_FLIP_OUT);
        } else {
            this.flip.setPosition(Constants.WOBBLE_FLIP_IN);
        }

        this.claw.setPosition(clawPos);

        if(liftPower > 0 && lift.getCurrentPosition() < liftHighLimit) {
            power = (Constants.LIFT_HIGH - lift.getCurrentPosition()) / Constants.LIMIT_RANGE * liftPower;
        }
        else if(liftPower < 0 && lift.getCurrentPosition() < Constants.LIMIT_RANGE) {
            power = (lift.getCurrentPosition() / Constants.LIMIT_RANGE) * liftPower;
        }
        else {
            power = liftPower;
        }

        lift.setPower(power * 0.75);

        Robot.telemetry.addData("lift position", lift.getCurrentPosition());

    }

    public double getLiftPosition() {
        return lift.getCurrentPosition();
    }

    @Override
    public void stop() {

    }
}
