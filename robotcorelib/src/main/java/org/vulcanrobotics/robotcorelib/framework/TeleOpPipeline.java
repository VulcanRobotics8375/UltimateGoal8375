package org.vulcanrobotics.robotcorelib.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.vulcanrobotics.robotcorelib.dashboard.Dashboard;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public abstract class TeleOpPipeline extends LinearOpMode {

    protected String ip;
    protected boolean dash;
    protected RobotConfig subsystems;

    public void teleopInit() {
        try {
            Robot.hardwareMap = hardwareMap;
            //comment out these lines in the competition version of the code
//            if(dash) {
//                Dashboard.connect(ip, 8375);
//                while (!Dashboard.running) {}
//            }

            //initialize backend Robot stuff
            Robot.setTelemetry(telemetry);
            Robot.init();
            subsystems = Robot.getComponents();
        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }
    }

    public void teleopStart() {
        if(dash) {
            Dashboard.start();
        }
//        Robot.startOdometryThread();
    }

    public void setStart(Point position, double angle) {
        Robot.setRobotPos(position);
        Robot.setRobotAngle(angle);
    }

    public void teleopStop() {
        Dashboard.running = false;
//        Robot.stopOdometryThread();
    }

}
