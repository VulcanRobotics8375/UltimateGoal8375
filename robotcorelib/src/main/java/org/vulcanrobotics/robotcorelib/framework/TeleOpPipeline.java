package org.vulcanrobotics.robotcorelib.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.vulcanrobotics.robotcorelib.dashboard.Dashboard;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public abstract class TeleOpPipeline extends OpMode {

    protected String ip;
    protected boolean dash;

    public abstract void init();

    public void teleopInit() {
        try {
            //comment out these 2 lines in the competition version of the code
            if(dash) {
                Dashboard.connect(ip, 8375);
                while (!Dashboard.running) {}
            }

            //initialize backend Robot stuff
            Robot.setTelemetry(telemetry);
            Robot.init();
        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }
    }

    public void teleopStart() {
        if(dash) {
            Dashboard.start();
        }
        Robot.startOdometryThread();
    }

    public void teleopLoop() {}

    public void teleopStop() {
        Dashboard.running = false;
        Robot.stopOdometryThread();
    }

}
