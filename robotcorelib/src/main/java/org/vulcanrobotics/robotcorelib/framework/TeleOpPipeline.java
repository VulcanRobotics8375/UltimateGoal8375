package org.vulcanrobotics.robotcorelib.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.vulcanrobotics.robotcorelib.robot.Robot;

public class TeleOpPipeline extends OpMode {


    @Override
    public void init() {
        try {
            Robot.setTelemetry(telemetry);
            Robot.init();
        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {

    }
}
