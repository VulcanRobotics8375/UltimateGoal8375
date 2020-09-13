package org.vulcanrobotics.robotcorelib.motion;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.vulcanrobotics.robotcorelib.dashboard.hardware.Odometer;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.math.Timer;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import java.io.FileOutputStream;
import java.io.IOException;

public class Mecanum extends MotionProfile {

    private Odometer left, right, horizontal;
    private double radius, wheelBase, ticksPerRev, horizontalRevsPerDeg, countsPerCm;

    private double lastLeftPos = 0, lastRightPos = 0, lastStrafePos = 0, lastTheta = 0;

    public Mecanum(int odometerNum, Odometer... odometer) throws RobotCoreLibException {
        super(odometerNum, odometer);
    }

    public void init() throws RobotCoreLibException {
        super.init();
        if(odometerNum != 3)
            throw new RobotCoreLibException("incorrect number of odometers on motion profile");

        left = getOdometerByName("left");
        right = getOdometerByName("right");
        horizontal = getOdometerByName("strafe");

        radius = Double.parseDouble(properties.getProperty("radius"));
        wheelBase = Double.parseDouble(properties.getProperty("wheelBase"));
        ticksPerRev = Double.parseDouble(properties.getProperty("tpr"));
        horizontalRevsPerDeg = Double.parseDouble(properties.getProperty("rpd-horizontal"));


    }

    public void update() {
        Point currentPos = Robot.getRobotPos();

        double vl = ((left.getPosition() - lastLeftPos) * ticksPerRev) / (radius * 2.0 * Math.PI) * Timer.getDelta();
        double vr = ((right.getPosition() - lastRightPos) * ticksPerRev) / (radius * 2.0 * Math.PI) * Timer.getDelta();

        double dTheta = (radius / wheelBase) * (vl - vr);
        double robotAngle = Robot.getRobotAngleRad() + dTheta;
        double vh = ((horizontal.getPosition() - lastStrafePos - (dTheta * horizontalRevsPerDeg)) * ticksPerRev) / (radius * 2.0 * Math.PI) * Timer.getDelta();
        double vf = (radius / 2.0) * (vl + vr);

        currentPos.x += (vf * Math.cos(Robot.getRobotAngleRad())) + (vh * Math.sin(Robot.getRobotAngleRad()));
        currentPos.y += (vf * Math.sin(Robot.getRobotAngleRad())) + (vh * Math.cos(Robot.getRobotAngleRad()));
        Robot.setRobotAngle(robotAngle);
        Robot.setRobotPos(currentPos);

        lastLeftPos = left.getPosition();
        lastRightPos = right.getPosition();
        lastStrafePos = horizontal.getPosition();
        lastTheta = robotAngle;

    }

    public void calibrate(double gain) {
        while(Robot.drivetrain.getZAngle() < 90) {
            Robot.drivetrain.move(0, gain);

        }

        try {
            String path = AppUtil.getInstance().getSettingsFile("robotconfig.properties").getAbsolutePath();

            //set up file streams for writing to the properties file
            FileOutputStream output = new FileOutputStream(path);


            double angle = Robot.drivetrain.getZAngle();
            double offset = Math.abs(left.getPosition()) + Math.abs(right.getPosition());
            double offsetPerDegree = offset / angle;
            double wheelBase = (2 * 90 * offsetPerDegree) / (Math.PI*(ticksPerRev * (radius * 2.0 * Math.PI)));
            double horizontalTicksPerDegree = horizontal.getPosition() / Math.toRadians(angle);
            properties.setProperty("wheelBase", Double.toString(wheelBase));
           properties.setProperty("rpd-horizontal", Double.toString(horizontalTicksPerDegree));
           properties.store(output, null);


            output.close();
        }
        catch(IOException e) {
            e.printStackTrace();
        }
    }

    public Odometer getLeft() {
        return left;
    }

    public void setLeft(Odometer left) {
        this.left = left;
    }

    public Odometer getRight() {
        return right;
    }

    public void setRight(Odometer right) {
        this.right = right;
    }

    public Odometer getHorizontal() {
        return horizontal;
    }

    public void setHorizontal(Odometer horizontal) {
        this.horizontal = horizontal;
    }

    public double getRadius() {
        return radius;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    public double getWheelBase() {
        return wheelBase;
    }

    public void setWheelBase(double wheelBase) {
        this.wheelBase = wheelBase;
    }

    public double getTicksPerRev() {
        return ticksPerRev;
    }

    public void setTicksPerRev(double ticksPerRev) {
        this.ticksPerRev = ticksPerRev;
    }

    public double getHorizontalRevsPerDeg() {
        return horizontalRevsPerDeg;
    }

    public void setHorizontalRevsPerDeg(double horizontalRevsPerDeg) {
        this.horizontalRevsPerDeg = horizontalRevsPerDeg;
    }

}
