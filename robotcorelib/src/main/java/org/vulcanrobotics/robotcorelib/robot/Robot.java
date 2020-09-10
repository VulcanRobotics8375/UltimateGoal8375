package org.vulcanrobotics.robotcorelib.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.MotionProfile;
import org.vulcanrobotics.robotcorelib.subsystem.Drivetrain;
import org.vulcanrobotics.robotcorelib.subsystem.Subsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * Generic Robot class -- requires a Drivetrain subsystem, but is modular beyond that point.
 */
public class Robot {

    private static Point robotPos = new Point();
    private static double robotAngle;
    private static List<Subsystem> subsystems = new ArrayList<>();
    public static Drivetrain drivetrain;
    public static MotionProfile motionProfile;
    public static RobotConfig config = new RobotConfig();
    public static Telemetry telemetry;

    public static Point getRobotPos() {
        return robotPos;
    }

    public static double getRobotAngleRad() {
        robotAngle = Math.toRadians(drivetrain.getZAngle());
        return robotAngle;
    }

    public static double getRobotAngleDeg() {
        robotAngle = Math.toRadians(drivetrain.getZAngle());
        return Math.toDegrees(robotAngle);
    }

    /**
     * this doesnt really work lmao
     */
    public static void setRobotAngle(double angle) {
       robotAngle = angle;
    }

    public static void setRobotPos(Point newPos) {
        robotPos.setPoint(newPos);
    }

    public static double getRobotX() {
        return robotPos.x;
    }

    public static double getRobotY() {
        return robotPos.y;
    }

    public static void init() throws RobotCoreLibException {


        config.init();
        subsystems = config.subsystems;
        motionProfile = config.motionProfile;

        for (Subsystem sub : subsystems) {
            sub.init();
            if(sub instanceof Drivetrain)
                drivetrain = (Drivetrain) sub;
        }

        config.setupMotionProfile();


    }

    public static void setTelemetry(Telemetry telemetry) {
        Robot.telemetry = telemetry;
    }

    public static List<Subsystem> getSubsystems() {

        return subsystems;
    }



}
