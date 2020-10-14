package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import static org.vulcanrobotics.robotcorelib.framework.Constants.FIELD_SIZE_CM;
import static org.vulcanrobotics.robotcorelib.framework.Constants.TILE_SIZE_CM;


public class Shooter extends Subsystem {

    @Override
    public void init() {

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
