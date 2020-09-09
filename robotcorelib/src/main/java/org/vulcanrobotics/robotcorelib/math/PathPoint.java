package org.vulcanrobotics.robotcorelib.math;

public class PathPoint {

    public double x;
    public double y;
    public double speed;
    public double turnSpeed;
    public double lookAhead;
    public double angle;

    public PathPoint() {}

    public PathPoint(PathPoint point) {
        this.x = point.x;
        this.y = point.y;
        this.speed = point.speed;
        this.turnSpeed = point.turnSpeed;
        this.lookAhead = point.lookAhead;
        this.angle = point.angle;

    }

    public PathPoint(double x, double y, double speed, double turnSpeed, double lookAhead) {
        this.x = x;
        this.y = y;
        this.speed = speed;
        this.turnSpeed = turnSpeed;
        this.lookAhead = lookAhead;
    }

    public PathPoint(double x, double y, double speed, double turnSpeed, double lookAhead, double angle) {
        this.x = x;
        this.y = y;
        this.speed = speed;
        this.turnSpeed = turnSpeed;
        this.lookAhead = lookAhead;
        this.angle = angle;
    }

    public PathPoint(double x, double y, double speed, double turnSpeed) {
        this(x, y, speed, turnSpeed, 0);
    }

    public Point toPoint(){
        return new Point(this.x, this.y);
    }

    public void setPoint(Point p) {
        this.x = p.x;
        this.y = p.y;
    }

    public void setPathPoint(PathPoint p) {
        this.x = p.x;
        this.y = p.y;
        this.speed = p.speed;
        this.turnSpeed = p.turnSpeed;
        this.lookAhead = p.lookAhead;
        this.angle = p.angle;
    }

}
