package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Subsystem {

    public HardwareMap hardwareMap;

    public Subsystem() {}

    public abstract void init();

    public abstract void stop();


}
