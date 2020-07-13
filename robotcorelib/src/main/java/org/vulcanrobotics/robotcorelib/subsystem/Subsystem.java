package org.vulcanrobotics.robotcorelib.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Subsystem {

    protected HardwareMap hardwareMap;

    public Subsystem() {}

    public abstract void init();

    public abstract void stop();


}
