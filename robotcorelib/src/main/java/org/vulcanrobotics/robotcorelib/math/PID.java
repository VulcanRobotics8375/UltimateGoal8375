package org.vulcanrobotics.robotcorelib.math;

public class PID {

    private final double Kp;
    private final double Ki;
    private final double Kd;

    private Timer timer = new Timer();

    private double lastError = 0;
    private double integral = 0;

    private double output;

    public PID(double Kp, double Ki, double Kd) {

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void run(double target, double value) {
       double timeElapsed = timer.getDelta();

        double error = target - value;
        integral += ((error + lastError) / 2);
        double derivative = (error - lastError);

        output = Kp * error + Ki * integral + Kd * derivative;

        lastError = error;
    }

    public void reset() {
        lastError = 0;
        integral = 0;
        output = 0;
    }

    public double getOutput() {
        return output;
    }

}
