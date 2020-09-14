package org.vulcanrobotics.robotcorelib.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Properties;

public abstract class Subsystem {

    public HardwareMap hardwareMap;

    public Subsystem() {}

    public abstract void init();

    public abstract void stop();


}
