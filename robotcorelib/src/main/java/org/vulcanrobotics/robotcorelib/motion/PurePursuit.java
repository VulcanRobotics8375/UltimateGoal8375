package org.vulcanrobotics.robotcorelib.motion;

import org.vulcanrobotics.robotcorelib.math.Functions;
import org.vulcanrobotics.robotcorelib.math.PID;
import org.vulcanrobotics.robotcorelib.math.PathPoint;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import java.util.ArrayList;

public class PurePursuit extends Controller {

    ArrayList<ArrayList<PathPoint>> sections;

    private PID turnPID = new PID(1, 1, 1);

    public PurePursuit(ArrayList<ArrayList<PathPoint>> sections) {
        this.sections = sections;
    }

    public void run() {
        for (ArrayList<PathPoint> section : sections) {
            while(Math.abs(Robot.getRobotX() - section.get(section.size() - 1).x) + Math.abs(Robot.getRobotY() - section.get(section.size() - 1).y) > 0.1) {
                PathPoint followPoint = findFollowPoint(section);
                moveToPoint(followPoint);
                if(stop)
                    break;
            }
        }
    }

    private PathPoint findFollowPoint(ArrayList<PathPoint> path) {
        PathPoint followPoint = path.get(0);

        ArrayList<Point> circleIntersections;

        for (int i = 0; i < path.size(); i++) {
            PathPoint start = path.get(i);
            PathPoint end = path.get(i + 1);

            if(path.indexOf(end) == path.size() - 1) {
                circleIntersections = Functions.lineCircleIntersectNoBoundingBox(start.toPoint(), end.toPoint(), end.lookAhead, Robot.getRobotPos());
            } else {
                circleIntersections = Functions.lineCircleIntersect(start.toPoint(), end.toPoint(), end.lookAhead, Robot.getRobotPos());
            }

            double closestAngle = Double.MAX_VALUE;
            for (Point intersection : circleIntersections) {
                double angle = Math.atan2(intersection.y - Robot.getRobotX(), intersection.x - Robot.getRobotY());
                double relativePointAngle = Math.atan2(end.y - start.y, end.x - start.x);
                double deltaAngle = Math.abs(Functions.angleWrap(angle - relativePointAngle));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followPoint.setPathPoint(end);
                    followPoint.setPoint(intersection);
                }
            }
        }

        currentPoint.setPathPoint(followPoint);

        return followPoint;
    }

    public void moveToPoint(PathPoint point) {
        double absoluteAngleToPoint = Math.atan2(point.y - Robot.getRobotY(), point.x - Robot.getRobotX());

        double robotAngleToPoint = point.angle - Robot.getRobotAngleRad();

        turnPID.run(point.angle, robotAngleToPoint);

        double turnSpeed = turnPID.getOutput();

        Robot.drivetrain.fieldCentricMove(Math.cos(absoluteAngleToPoint) * point.speed, Math.sin(absoluteAngleToPoint) * point.speed, turnSpeed);
    }

}
