package org.vulcanrobotics.robotcorelib.math;

public class PID {

    private double Kp;
    private double Ki;
    private double Kd;

//    private Timer timer = new Timer();

    private double lastError = 0;
    private double integral = 0;
    private double derivative;
    private double lastValue;
    private double tau;
    private double loopTime;

    private double output;

    public PID(double Kp, double Ki, double Kd, double tau, double loopTime) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.tau = tau;
        this.loopTime = loopTime;
    }

    public void run(double target, double value) {

        double error = target - value;
        double proportional = Kp * error;
        integral += ((error + lastError) / 2.0) * loopTime;
        derivative = (2.0 * Kd * (value - lastValue) * ((2.0 * tau) - loopTime) * derivative) / ((2.0 * tau) + loopTime);

        output = proportional + integral + derivative;

        lastError = error;
    }

    public void reset() {
        lastError = 0;
        integral = 0;
        output = 0;
        derivative = 0;
        lastValue = 0;
    }

    public double getOutput() {
        return output;
    }

    public double getKp() {
        return Kp;
    }

    public void setKp(double kp) {
        Kp = kp;
    }

    public double getKi() {
        return Ki;
    }

    public void setKi(double ki) {
        Ki = ki;
    }

    public double getKd() {
        return Kd;
    }

    public void setKd(double kd) {
        Kd = kd;
    }
}
