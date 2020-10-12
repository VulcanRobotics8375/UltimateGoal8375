package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.vulcanrobotics.robotcorelib.framework.Constants;

import static org.vulcanrobotics.robotcorelib.framework.Constants.*;

public class Intake extends Subsystem {

    private DcMotor left, right;

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("intake_left");
        right = hardwareMap.dcMotor.get("intake_right");
    }

    //feel free to delete this it's just for quick testing stuff
    public void run(boolean on, boolean reverse, boolean transferOn, boolean transferReverse) {
        if(on) {
            left.setPower(INTAKE_LEFT_POWER);
        }
        else if(reverse){
            left.setPower(INTAKE_LEFT_POWER * -1.0);
        }
        else {
            left.setPower(0);
        }

        if(transferOn) {
            right.setPower(INTAKE_RIGHT_POWER);
        } else if(transferReverse) {
            right.setPower(INTAKE_RIGHT_POWER * -1.0);
        } else {
            right.setPower(0);
        }
    }

    @Override
    public void stop() {

    }
}
