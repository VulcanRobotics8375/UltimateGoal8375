package org.vulcanrobotics.robotcorelib.dashboard.hardware;

import org.vulcanrobotics.robotcorelib.dashboard.Dashboard;

/**
 * The Sensor is an abstract way to interface a value with the Dashboard
 */
public class Sensor {
    public String name;
    public double value;

    public Sensor() {}

    public Sensor(String name, double value) {
        this.name = name;
        this.value = value;

        Dashboard.addSensor(this);
    }

    public void update() {
        Dashboard.sendToDash("/update ");

    }
}
