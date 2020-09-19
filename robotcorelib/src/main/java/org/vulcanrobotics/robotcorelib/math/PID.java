package org.vulcanrobotics.robotcorelib.math;

public class PID {

    private final double Kp;
    private final double Ki;
    private final double Kd;

    private double lastError = 0;
    private double integral = 0;
    private long lastTime;

    private double output;

    public PID(double Kp, double Ki, double Kd) {

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void run(double target, double value) {
        double timeElapsed = System.nanoTime() - lastTime;
        lastTime = System.nanoTime();

        double error = target - value;
        integral += ((error + lastError) / 2) * timeElapsed;
        double derivative = (error - lastError) * timeElapsed;

        output = Kp * error + Ki * integral + Kd * derivative;

        lastError = error;
    }

    public double getOutput() {
        return output;
    }

}
