package org.vulcanrobotics.robotcorelib.dashboard;

/**
 * Very similar to Sensor, just sorted differently when it gets to the Dashboard.
 * @see org.vulcanrobotics.robotcorelib.dashboard.hardware.Sensor
 */
public class Constant {
    public String key;
    public Number val;

    public Constant() {}

    public Constant(String key, Number val) {
        this.key = key;
        this.val = val;
    }

//    public void update() {
//        Dashboard.sendToDash("/update Constant " + key + " " + val);
//    }
}
