package org.vulcanrobotics.robotcorelib.Dashboard.Hardware;

import org.vulcanrobotics.robotcorelib.Dashboard.Dashboard;

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
