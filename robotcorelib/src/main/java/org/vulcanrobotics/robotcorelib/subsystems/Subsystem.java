package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Subsystem {

    public HardwareMap hardwareMap;

    protected Telemetry telemetry;

    public Subsystem() {}

    public abstract void init();

    public abstract void stop();

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

}
