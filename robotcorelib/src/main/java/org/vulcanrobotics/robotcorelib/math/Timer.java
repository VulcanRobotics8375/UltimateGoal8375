package org.vulcanrobotics.robotcorelib.math;

public class Timer {

    public Timer() {

    }

    private  long lastTime = 0;

    public void init() {
        lastTime = System.nanoTime();
    }

    public  long getDelta() {
        long time = System.nanoTime();
        long delta = time - lastTime;
        lastTime = time;
        return delta;
    }

}
