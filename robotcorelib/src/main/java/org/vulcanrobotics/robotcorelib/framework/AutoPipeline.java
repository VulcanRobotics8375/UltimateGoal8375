package org.vulcanrobotics.robotcorelib.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.Controller;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public abstract class AutoPipeline extends LinearOpMode {

    protected Controller controller;

    public abstract void runOpMode();

    public void autoInit() throws RobotCoreLibException {
        Robot.setTelemetry(telemetry);
        Robot.init();
    }

    public void setStart(Point position, double angle) {
        Robot.setRobotPos(position);
        Robot.setRobotAngle(angle);
    }

    public void startInterruptHandler() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                while(!isStopRequested()) {}
               controller.stop = isStopRequested();
            }
        }).start();
    }

}
