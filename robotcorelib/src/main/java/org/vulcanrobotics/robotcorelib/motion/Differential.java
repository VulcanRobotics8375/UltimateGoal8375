package org.vulcanrobotics.robotcorelib.motion;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.vulcanrobotics.robotcorelib.dashboard.hardware.Odometer;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import java.io.FileOutputStream;
import java.io.IOException;

public class Differential extends MotionProfile {

    private Odometer left;
    private Odometer right;
    private Point lastPos;

    private double radius;
    private double wheelBase;
    private double ticksPerRev;

    private double lastLeftPosition;
    private double lastRightPosition;

    public Differential(int odometerNum, Odometer... odometer) throws RobotCoreLibException {
        super(odometerNum, odometer);
    }

    public void init() throws RobotCoreLibException {
        super.init();
        if(odometerNum != 2)
            throw new RobotCoreLibException("incorrect number of odometers on motion profile");
        left = getOdometerByName("left");
        right = getOdometerByName("right");

        radius = Double.parseDouble(properties.getProperty("radius"));
        wheelBase = Double.parseDouble(properties.getProperty("wheelBase"));
        ticksPerRev = Double.parseDouble(properties.getProperty("tpr"));

    }

    public void update() {
        Point currentPos = Robot.getRobotPos();

        double vl = (left.getPosition() - lastLeftPosition) / ticksPerRev;
        double vr = (right.getPosition() - lastRightPosition) / Double.parseDouble(properties.getProperty("tpr"));

        double v = (radius / 2.0) * (vl + vr);

        currentPos.x += v * Math.cos(Robot.getRobotAngleRad());
        currentPos.y += v * Math.sin(Robot.getRobotAngleRad());

        lastLeftPosition = left.getPosition();
        lastRightPosition = right.getPosition();
        lastPos.setPoint(currentPos);
        Robot.setRobotPos(currentPos);

    }

    public void calibrate(double gain) {
        while(Robot.getRobotAngleDeg() < 90) {
            Robot.drivetrain.move(0, gain);
        }

        try {
            String path = AppUtil.getInstance().getSettingsFile("robotconfig.properties").getAbsolutePath();

            //set up file streams for writing to the properties file
            FileOutputStream output = new FileOutputStream(path);

            //get current constants
            double radius = Double.parseDouble(properties.getProperty("radius"));
            double ticksPerRotation = Double.parseDouble(properties.getProperty("tpr"));
            double driftTolerance = Double.parseDouble(properties.getProperty("driftTolerance"));

            //make sure encoders dont drift too much
            double distanceTravelledRight = (right.getPosition() / ticksPerRotation) * radius;
            double distanceTravelledLeft = (left.getPosition() / ticksPerRotation) * radius;
            if(Math.abs(distanceTravelledLeft - distanceTravelledRight) > driftTolerance)
                throw new RobotCoreLibException("calibration error- too much difference between encoders");

            //set wheelBase, the only constant that can really be calibrated here
            double wheelBase = distanceTravelledLeft / Math.toRadians(90);
            properties.setProperty("wheelBase", String.valueOf(wheelBase));

            //store the properties file with new constants
            properties.store(output, null);

        } catch (IOException | RobotCoreLibException e) {
            e.printStackTrace();
        }
    }

}
