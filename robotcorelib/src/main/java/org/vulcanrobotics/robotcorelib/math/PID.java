package org.vulcanrobotics.robotcorelib.math;

import com.qualcomm.robotcore.util.Range;

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
    private double limMin, limMax;

    private double output;

    public PID(double Kp, double Ki, double Kd, double tau, double loopTime, double limMin, double limMax) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.tau = tau;
        this.loopTime = loopTime;
        this.limMin = limMin;
        this.limMax = limMax;
    }

    public void run(double target, double value) {

        double error = target - value;
        double proportional = Kp * error;
        integral += ((error + lastError) / 2.0) * loopTime;

        double limMinInt, limMaxInt;
        if(proportional < limMax) {
            limMaxInt = limMax;
        } else {
            limMaxInt = 0.0;
        }
        if(proportional > limMin) {
            limMinInt = limMin;
        } else {
            limMinInt = 0.0;
        }

        integral = Range.clip(integral, limMinInt, limMaxInt);

        derivative = (2.0 * Kd * (value - lastValue) * ((2.0 * tau) - loopTime) * derivative) / ((2.0 * tau) + loopTime);

        output = proportional + integral + derivative;

        lastError = error;
        lastValue = value;
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

    public double getTau() {
        return tau;
    }

    public void setTau(double tau) {
        this.tau = tau;
    }

    public double getLoopTime() {
        return loopTime;
    }

    public void setLoopTime(double loopTime) {
        this.loopTime = loopTime;
    }

    public double getLimMin() {
        return limMin;
    }

    public void setLimMin(double limMin) {
        this.limMin = limMin;
    }

    public double getLimMax() {
        return limMax;
    }

    public void setLimMax(double limMax) {
        this.limMax = limMax;
    }

}
