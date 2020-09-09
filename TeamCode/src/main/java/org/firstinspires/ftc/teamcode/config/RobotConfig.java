package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.vulcanrobotics.robotcorelib.subsystem.Drivetrain;
import org.vulcanrobotics.robotcorelib.subsystem.Subsystem;

import java.util.ArrayList;
import java.util.List;

public class RobotConfig {

    public Drivetrain drivetrain = new Drivetrain();
    public Shooter shooter = new Shooter();

    public List<Subsystem> subsystems = new ArrayList<>();
    public void init() {
        subsystems.add(drivetrain);
        subsystems.add(shooter);

    }

}
