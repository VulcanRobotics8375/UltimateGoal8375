package org.vulcanrobotics.robotcorelib.framework;

import org.vulcanrobotics.robotcorelib.dashboard.hardware.Odometer;
import org.vulcanrobotics.robotcorelib.motion.Mecanum;
import org.vulcanrobotics.robotcorelib.motion.MotionProfile;
import org.vulcanrobotics.robotcorelib.subsystems.Drivetrain;
import org.vulcanrobotics.robotcorelib.subsystems.Intake;
import org.vulcanrobotics.robotcorelib.subsystems.Shooter;
import org.vulcanrobotics.robotcorelib.subsystems.Subsystem;
import org.vulcanrobotics.robotcorelib.subsystems.WobbleGrabber;

import java.util.ArrayList;
import java.util.List;

public class RobotConfig {

    public MotionProfile motionProfile;

    public Drivetrain drivetrain = new Drivetrain();
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    public WobbleGrabber wobbleGrabber = new WobbleGrabber();

    private List<Subsystem> subsystems = new ArrayList<>();

    public RobotConfig() {
    }

    public void init() {
        subsystems.clear();
        subsystems.add(drivetrain);
        subsystems.add(shooter);
        subsystems.add(intake);
//        subsystems.add(wobbleGrabber);

    }

    public void setupMotionProfile() throws RobotCoreLibException {
        Odometer left = new Odometer("left", drivetrain.getFrontLeft());
        Odometer right = new Odometer("right", drivetrain.getFrontRight());
        Odometer strafe = new Odometer("strafe", drivetrain.getBackRight());

        motionProfile = new Mecanum(3, left, right, strafe);
        motionProfile.init();
    }

    public List<Subsystem> getSubsystems() {
        return subsystems;
    }

}
