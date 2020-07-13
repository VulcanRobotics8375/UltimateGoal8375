package org.vulcanrobotics.robotcorelib.robot;

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

    private static Point robotPos;
    private static double robotAngle;
    private static List<Subsystem> subsystems = new ArrayList<>();
    public static Drivetrain drivetrain;
    public static MotionProfile motionProfile;

    public static void addSubsystem(Subsystem sub) {
        subsystems.add(sub);
    }

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

    public static void init() {
        for (Subsystem sub : subsystems) {
            sub.init();
            if(sub instanceof Drivetrain)
                drivetrain = (Drivetrain) sub;
        }

    }

    public static void move(double fl, double fr, double bl, double br) {

    }

}
