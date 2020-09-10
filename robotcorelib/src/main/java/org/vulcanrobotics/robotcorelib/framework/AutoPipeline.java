package org.vulcanrobotics.robotcorelib.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.vulcanrobotics.robotcorelib.robot.Robot;

public abstract class AutoPipeline extends LinearOpMode {

    public abstract void runOpMode();

    public void autoInit() throws RobotCoreLibException {
        Robot.setTelemetry(telemetry);
        Robot.init();
    }


}
