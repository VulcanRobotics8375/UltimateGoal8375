package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.vulcanrobotics.robotcorelib.framework.ConstantParser;
import org.vulcanrobotics.robotcorelib.robot.Robot;
import org.vulcanrobotics.robotcorelib.subsystem.Subsystem;

/**
 * includes conveyor belt
 */
public class Intake extends Subsystem {

    private Servo left;
    private Servo right;
    private Servo flip;

    private DcMotor conveyor;

    private boolean flipping;

    @Override
    public void init() {

        left = hardwareMap.servo.get("intake_left");
        right = hardwareMap.servo.get("intake_right");
        flip = hardwareMap.servo.get("intake_flip");

        conveyor = hardwareMap.dcMotor.get("conveyor");

    }

    public void run(boolean claw, boolean flip) {

        if(claw) {
            setClawPosition(ConstantParser.parseDouble("clawIn"));
        }
        else {
            setClawPosition(ConstantParser.parseDouble("clawOut"));
        }

    }

    private void setClawPosition(double position) {
        left.setPosition(position);
        right.setPosition(-position);
    }

    @Override
    public void stop() {

    }
}
