package org.vulcanrobotics.robotcorelib.motion;

import org.vulcanrobotics.robotcorelib.math.PathPoint;

public abstract class Controller {

    public volatile boolean stop = false;

    protected PathPoint currentPoint = new PathPoint();

    public abstract void run();

    public PathPoint getCurrentPoint() {
        return currentPoint;
    }

}
