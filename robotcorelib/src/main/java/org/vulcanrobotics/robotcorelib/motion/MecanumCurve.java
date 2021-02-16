package org.vulcanrobotics.robotcorelib.motion;

import org.vulcanrobotics.robotcorelib.dashboard.hardware.Odometer;
import org.vulcanrobotics.robotcorelib.framework.Constants;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public class MecanumCurve extends MotionProfile {

    private Odometer left, right, horizontal;
    private double radius, wheelBase, ticksPerRev, horizontalTicksPerDeg, countsPerCm, horizontalDistanceFromCenter;

    private double lastLeftPos = 0, lastRightPos = 0, lastStrafePos = 0, lastTheta = 0;

    public MecanumCurve(int odometerNum, Odometer... odometer) throws RobotCoreLibException {
        super(odometerNum, odometer);
    }

    public void init() throws RobotCoreLibException {
        super.init();
        if(odometerNum != 3)
            throw new RobotCoreLibException("incorrect number of odometers on motion profile");

        left = getOdometerByName("left");
        right = getOdometerByName("right");
        horizontal = getOdometerByName("strafe");

        radius = Constants.ODOMETRY_RADIUS;
        wheelBase = Constants.ODOMETRY_WHEELBASE;
        ticksPerRev = Constants.ODOMETRY_TICKS_PER_REV;
        horizontalTicksPerDeg = Constants.ODOMETRY_HORIZONTAL_TICKS_PER_RAD;
        countsPerCm = Constants.ODOMETRY_COUNTS_PER_CM;
        //TODO add constant to Constants.java
        horizontalDistanceFromCenter = 1.0;


    }

    public synchronized void update() {
        Point currentPos = Robot.getRobotPos();

        double leftPosition = left.getPosition();
        double rightPosition = right.getPosition();
        double horizontalPosition = horizontal.getPosition();

        double leftChange = (leftPosition - lastLeftPos);
        double rightChange = (rightPosition - lastRightPos);
        double rawHorizontalChange = (horizontalPosition - lastStrafePos);
        double thetaChange = (leftChange - rightChange) / (wheelBase * countsPerCm);

        double verticalArcLength = (leftChange + rightChange) / 2;
        double horizontalChange;
        double verticalChange;
        if(thetaChange == 0) {
            horizontalChange = rawHorizontalChange;
            verticalChange = verticalArcLength;
        } else {
            double turnRadius = ((wheelBase / 2.0) * countsPerCm * (leftChange + rightChange)) / (rightChange - leftChange);
            double strafeRadius = (rawHorizontalChange / thetaChange) - (horizontalDistanceFromCenter * countsPerCm);
            horizontalChange = (turnRadius * Math.cos(thetaChange - 1)) + (strafeRadius * Math.sin(thetaChange));
            verticalChange = (turnRadius * Math.sin(thetaChange)) + (strafeRadius * (1 - Math.cos(thetaChange)));

        }

        double robotAngle = Robot.getRobotAngleRad();

        currentPos.x += ((verticalChange * Math.sin(robotAngle + (thetaChange / 2.0))) + (horizontalChange * Math.cos(robotAngle + (thetaChange / 2.0)))) * (1 / countsPerCm);
        currentPos.y += ((verticalChange * Math.cos(robotAngle + (thetaChange / 2.0))) + (horizontalChange * Math.sin(robotAngle + (thetaChange / 2.0)))) * (1 / countsPerCm);

        lastLeftPos = leftPosition;
        lastRightPos = rightPosition;
        lastStrafePos = horizontalPosition;
        lastTheta = robotAngle + thetaChange;
        Robot.setRobotPos(currentPos);
        Robot.setRobotAngle(lastTheta);
    }

    public void calibrate(double gain) {
        while(Robot.drivetrain.getZAngle() < 90) {
            Robot.drivetrain.move(0, gain);

        }
        Robot.drivetrain.move(0, 0);
        long lastTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - lastTime < 1000) {}

        double angle = Robot.drivetrain.getZAngle();
        double offset = Math.abs(left.getPosition()) + Math.abs(right.getPosition());
        double offsetPerDegree = offset / angle;
        double wheelBase = (2.0 * 90 * offsetPerDegree) / (Math.PI*(countsPerCm));
        double horizontalTicksPerDegree = horizontal.getPosition() / angle;

        Robot.telemetry.addData("wheelBase", wheelBase);
        Robot.telemetry.addData("horizontal per deg", horizontalTicksPerDegree);
        Robot.telemetry.addData("left encoder", left.getPosition());
        Robot.telemetry.addData("right encoder", right.getPosition());
        Robot.telemetry.addData("horizontal encoder", horizontal.getPosition());
        Robot.telemetry.addData("angle", angle);

        Robot.telemetry.update();
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
        return horizontalTicksPerDeg;
    }

    public void setHorizontalRevsPerDeg(double horizontalRevsPerDeg) {
        this.horizontalTicksPerDeg = horizontalRevsPerDeg;
    }

}
