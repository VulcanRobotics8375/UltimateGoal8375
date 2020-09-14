package org.vulcanrobotics.robotcorelib.framework;

import org.vulcanrobotics.robotcorelib.dashboard.hardware.Odometer;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.motion.Mecanum;
import org.vulcanrobotics.robotcorelib.motion.MotionProfile;
import org.vulcanrobotics.robotcorelib.subsystem.Drivetrain;
import org.vulcanrobotics.robotcorelib.subsystem.Intake;
import org.vulcanrobotics.robotcorelib.subsystem.Shooter;
import org.vulcanrobotics.robotcorelib.subsystem.Subsystem;
import org.vulcanrobotics.robotcorelib.subsystem.WobbleGrabber;

import java.util.ArrayList;
import java.util.List;

public class RobotConfig {

    public MotionProfile motionProfile;

    public Drivetrain drivetrain = new Drivetrain();
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    public WobbleGrabber wobbleGrabber = new WobbleGrabber();

    public List<Subsystem> subsystems = new ArrayList<>();

    public RobotConfig() {
    }

    public void init() {
        subsystems.add(drivetrain);
        subsystems.add(shooter);
        subsystems.add(intake);
        subsystems.add(wobbleGrabber);

    }

    public void setupMotionProfile() throws RobotCoreLibException {
        Odometer left = new Odometer("left", drivetrain.getBackLeft());
        Odometer right = new Odometer("right", drivetrain.getBackRight());
        Odometer strafe = new Odometer("strafe", drivetrain.getFrontLeft());

        motionProfile = new Mecanum(3, left, right, strafe);
        motionProfile.init();
    }

}
