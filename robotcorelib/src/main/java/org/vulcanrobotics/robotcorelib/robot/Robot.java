package org.vulcanrobotics.robotcorelib.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.vulcanrobotics.robotcorelib.drive.StandardTrackingWheelLocalizer;
import org.vulcanrobotics.robotcorelib.framework.RobotConfig;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.math.Timer;
import org.vulcanrobotics.robotcorelib.motion.MotionProfile;
import org.vulcanrobotics.robotcorelib.subsystems.Drivetrain;
import org.vulcanrobotics.robotcorelib.subsystems.Subsystem;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Properties;

/**
 * Generic Robot class -- requires a Drivetrain subsystem, but is modular beyond that point.
 */
public class Robot {

    /**
     * Robot Position
     */
    private static volatile Point robotPos = new Point();
    private static volatile Point robotVelocity = new Point();
    private static Point resetPosition = new Point();
    /**
     * robot angle in Radians
     */
    private static volatile double robotAngle = 0;
    /**
     * Drivetrain subsystem, automatically assigned during init()
     */
    public static Drivetrain drivetrain;
    /**
     * Generic MotionProfile object so the robot class can start/stop odometry
     */
    public static MotionProfile motionProfile;
    /**
     * Stores all subsystems and motion profiles
     */
    public static RobotConfig config = new RobotConfig();
    /**
     * Telemetry Object, inhereted from the currently running OpMode
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode
     */
    public static Telemetry telemetry;
    /**
     * HardwareMap Object inherited from the currently running OpMode
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode
     * @see org.vulcanrobotics.robotcorelib.framework.TeleOpPipeline
     * @see org.vulcanrobotics.robotcorelib.framework.AutoPipeline
     */
    public static HardwareMap hardwareMap;

    private static Properties internalProperties;


    /**
     * Thread-safe boolean for starting/stopping the odometry thread.
     */
    private static volatile boolean odometryRunning;

    /**
     *
     * @return Returns the Robot's global position, updated by the MotionProfile
     * @see MotionProfile
     */
    public static Point getRobotPos() {
        return robotPos;
    }

    /**
     *
     * @return Returns the Robots global angle, in radians, updated by the MotionProfile
     * @see MotionProfile
     */
    public static double getRobotAngleRad() {
        return robotAngle;
    }

    /**
     *
     * @return Returns the Robot's global angle in degrees, updated by the MotionProfile
     * @see MotionProfile
     */
    public static double getRobotAngleDeg() {
       return Math.toDegrees(robotAngle);
    }

    /**
     * Sets the robot's global angle.
     * @param angle robot angle in radians
     */
    public static void setRobotAngle(double angle) {
       robotAngle = angle;
    }

    /**
     * Sets the robot's global position, in cm
     * @param newPos Point object with the new position (x,y)
     */
    public static void setRobotPos(Point newPos) {
        robotPos.setPoint(newPos);
    }

    /**
     *
     * @return Returns just the robot's global X coordinate, in cm
     */
    public static double getRobotX() {
        return robotPos.x;
    }

    /**
     *
     * @return Returns just the robot's global Y coordinate, in cm
     */
    public static double getRobotY() {
        return robotPos.y;
    }

    public static double getRobotXVelocity() {
        return robotVelocity.x;
    }

    public static double getRobotYVelocity() {
        return robotVelocity.y;
    }

    public static void setRobotVelocity(Point velocity) {
        robotVelocity.setPoint(velocity);
    }

    public static Point getRobotVelocity() {
        return robotVelocity;
    }

    public static void resetRobotPosition() {
        robotPos.setPoint(resetPosition);
        robotAngle = 0;
    }

    public static void setResetPosition(Point reset) {
        resetPosition.setPoint(reset);
    }

    /**
     * The init() method initializes all subsystems, motion profiles, etc. This is run during an OpMode's INIT period.
     * @throws RobotCoreLibException whenever something goes wrong
     */
    public static void init() throws RobotCoreLibException  {

        config.init();
        List<Subsystem> subsystems = config.getSubsystems();

        for (Subsystem sub : subsystems) {
//            telemetry.addData("initializing subsystem:", sub.toString());
//            telemetry.update();
            sub.hardwareMap = hardwareMap;
            sub.setTelemetry(telemetry);
            sub.init();
            if(sub instanceof Drivetrain)
                drivetrain = (Drivetrain) sub;
        }

        config.setupMotionProfile();
        motionProfile = config.motionProfile;

    }

    public static void test() {}

    public static void init(HardwareMap hardwareMap, Telemetry telemetry) throws RobotCoreLibException {
        Robot.hardwareMap = hardwareMap;
        Robot.setTelemetry(telemetry);
        Robot.init();
    }

    public static MotionProfile getMotionProfile() {
        return motionProfile;
    }

    public static void setTelemetry(Telemetry telemetry) {
        Robot.telemetry = telemetry;
    }

    public static RobotConfig getComponents() {

        return config;
    }

    public static Properties getPropertiesFile() {
        return internalProperties;
    }

    public static void storeRobotPosition() {
        File xPosFile = AppUtil.getInstance().getSettingsFile("xPos.txt");
        File yPosFile = AppUtil.getInstance().getSettingsFile("yPos.txt");
        File thetaFile = AppUtil.getInstance().getSettingsFile("theta.txt");

        ReadWriteFile.writeFile(xPosFile, String.valueOf(getRobotX()));
        ReadWriteFile.writeFile(yPosFile, String.valueOf(getRobotY()));
        ReadWriteFile.writeFile(thetaFile, String.valueOf(getRobotAngleRad()));

    }

    public static void loadRobotPosition() {
        File xPosFile = AppUtil.getInstance().getSettingsFile("xPos.txt");
        File yPosFile = AppUtil.getInstance().getSettingsFile("yPos.txt");
        File thetaFile = AppUtil.getInstance().getSettingsFile("theta.txt");

        double x = Double.parseDouble(ReadWriteFile.readFile(xPosFile).trim());
        double y = Double.parseDouble(ReadWriteFile.readFile(yPosFile).trim());
        double theta = Double.parseDouble(ReadWriteFile.readFile(thetaFile).trim());

        setRobotPos(new Point(x, y));
        setRobotAngle(theta);

    }

    /**
     * starts a new Thread that runs the MotionProfile's <code>update()</code> method until <code>odometryRunning</code> is false.
     * @see Thread
     * @see Runnable
     */
    public synchronized static void startOdometryThread() {
        odometryRunning = true;
        new Thread(new Runnable() {
            @Override
            public void run() {
                StandardTrackingWheelLocalizer drive = new StandardTrackingWheelLocalizer(hardwareMap);
                while(odometryRunning) {
                    drive.update();
                    Pose2d poseEstimate = drive.getPoseEstimate();
                    Pose2d poseVelocity = drive.getPoseVelocity();

                    setRobotPos(new Point(poseEstimate.getY(), poseEstimate.getX()));
                    setRobotAngle(poseEstimate.getHeading());
                    setRobotVelocity(new Point(poseVelocity.getY(), poseVelocity.getX()));
                }
            }
        }).start();
    }

    /**
     * sets <code>odometryRunning</code> to false, stopping the odometry thread if running.
     */
    public static void stopOdometryThread() {
        odometryRunning = false;
    }



}
