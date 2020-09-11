package org.vulcanrobotics.robotcorelib.motion;

public abstract class Controller {

    public volatile boolean stop = false;

    public abstract void run();

}
