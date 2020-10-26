package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import static org.vulcanrobotics.robotcorelib.framework.Constants.FIELD_SIZE_CM;
import static org.vulcanrobotics.robotcorelib.framework.Constants.TILE_SIZE_CM;


public class Shooter extends Subsystem {
    private DcMotor flywheel;
    private Servo conveyor;

    @Override
    public void init() {
        flywheel = hardwareMap.dcMotor.get("flywheel");
        conveyor = hardwareMap.servo.get("conveyor");
    }

    public void run(boolean flywheelOn, boolean conveyorOn) {

        if(flywheelOn) {
            flywheel.setPower(-0.8);
        } else {
            flywheel.setPower(0.0);
        }

        if(conveyorOn) {
            conveyor.setPosition(0.3);
        }
        else {
            conveyor.setPosition(0.0);
        }

    }

    /**
     * this is going to be after all of our spreadsheet data, might use a spreadsheet parser
     * @return returns shooter power in range (0.0, 1.0]
     */
    private double calcShooterPower() {
        double distanceToTarget = Math.hypot(1.5*TILE_SIZE_CM - Robot.getRobotX(), FIELD_SIZE_CM * Robot.getRobotY());

        return 0;

    }

    @Override
    public void stop() {

    }

}
