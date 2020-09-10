package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.vulcanrobotics.robotcorelib.dashboard.hardware.Odometer;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.motion.Mecanum;
import org.vulcanrobotics.robotcorelib.motion.MotionProfile;
import org.vulcanrobotics.robotcorelib.subsystem.Drivetrain;
import org.vulcanrobotics.robotcorelib.subsystem.Subsystem;

import java.util.ArrayList;
import java.util.List;

public class RobotConfig {

    public MotionProfile motionProfile;

    public Drivetrain drivetrain = new Drivetrain();
    public Shooter shooter = new Shooter();

    public List<Subsystem> subsystems = new ArrayList<>();

    public RobotConfig() {
    }

    public void init() {
        subsystems.add(drivetrain);
        subsystems.add(shooter);

    }

    public void setupMotionProfile() throws RobotCoreLibException {
        Odometer left = new Odometer("left", drivetrain.getBackLeft().getMotor());
        Odometer right = new Odometer("right", drivetrain.getBackRight().getMotor());
        Odometer strafe = new Odometer("strafe", drivetrain.getFrontLeft().getMotor());

        motionProfile = new Mecanum(3, left, right, strafe);
        motionProfile.init();
    }

}
