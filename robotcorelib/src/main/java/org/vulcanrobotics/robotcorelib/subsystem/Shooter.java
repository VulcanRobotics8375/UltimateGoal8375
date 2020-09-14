package org.vulcanrobotics.robotcorelib.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.vulcanrobotics.robotcorelib.dashboard.hardware.DashboardMotor;
import org.vulcanrobotics.robotcorelib.dashboard.hardware.DashboardServo;
import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.subsystem.Subsystem;


public class Shooter extends Subsystem {

    private DcMotor conveyor;
    private DcMotor flywheel;

    @Override
    public void init() {

        conveyor = hardwareMap.dcMotor.get("conveyor");
        flywheel = hardwareMap.dcMotor.get("flywheel");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void run(boolean conveyorOn, double shooterPower) {

        if(conveyorOn) {
            conveyor.setPower(Constants.CONVEYOR_POWER);
        } else {
            conveyor.setPower(0.0);
        }

        flywheel.setPower(shooterPower);

    }

    double testPower = 0;
    boolean testUp;
    boolean testDown;

    public void test(boolean up, boolean down, boolean on, boolean conveyorOn) {
        if(up && !testUp) {
            testPower += 0.1;
            testUp = true;
        }
        if(!up && testUp) {
            testUp = false;
        }

        if(down && !testDown) {
            testPower -= 0.1;
            testDown = true;
        }
        if(!down && testDown) {
            testDown = false;
        }

        if(on) {
            flywheel.setPower(testPower);
        }
        if(conveyorOn) {
            conveyor.setPower(1.0);
        } else {
            conveyor.setPower(0);
        }

    }

    @Override
    public void stop() {

    }


    public void setServoAngle(Servo servo, double angle, double maxAngle) {
        servo.setPosition(angle / maxAngle);
    }
}
