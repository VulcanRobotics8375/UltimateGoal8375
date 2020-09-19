package org.vulcanrobotics.robotcorelib.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public abstract class Subsystem {

    public HardwareMap hardwareMap;

    protected Telemetry telemetry = Robot.telemetry;

    public Subsystem() {}

    public abstract void init();

    public abstract void stop();


}
