package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public class OpModeOne extends OpMode {
    @Override
    public void init() {
        try {
            Robot.init();
        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {

    }
}
