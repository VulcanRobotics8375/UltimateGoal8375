package org.vulcanrobotics.robotcorelib.Dashboard;

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
