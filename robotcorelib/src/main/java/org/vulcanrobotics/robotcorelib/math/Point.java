package org.vulcanrobotics.robotcorelib.math;

public class Point {
    public double x;
    public double y;

    public Point(Point point) {
        this.x = point.x;
        this.y = point.y;
    }

    public Point() {}

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setPoint(Point point) {
        this.x = point.x;
        this.y = point.y;
    }

}
