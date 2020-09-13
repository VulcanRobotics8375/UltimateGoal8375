package org.vulcanrobotics.robotcorelib.math;

public class Timer {

    private static long lastTime = 0;

    public static long getDelta() {
        long time = System.nanoTime();
        long delta = time - lastTime;
        lastTime = time;
        return delta;
    }

}
