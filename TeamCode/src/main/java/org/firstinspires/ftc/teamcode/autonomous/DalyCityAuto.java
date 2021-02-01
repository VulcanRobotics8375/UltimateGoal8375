package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.vulcanrobotics.robotcorelib.framework.AutoPipeline;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.math.PathPoint;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.PurePursuit;
import org.vulcanrobotics.robotcorelib.robot.Robot;
import org.vulcanrobotics.robotcorelib.vision.StackDetectorCV;

import java.util.ArrayList;

@Autonomous(name = "new auto", group = "main")
public class DalyCityAuto extends AutoPipeline {
    @Override
    public void runOpMode() {
        try {
            autoInit();
        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }
        subsystems.drivetrain.autoInit();
        StackDetectorCV stackDetector = new StackDetectorCV();
        initVision(stackDetector);
        subsystems.wobbleGrabber.wobbleGrab.setPosition(1.5);

        setStart(new Point(138.34, 21.6), 0.0);

        ArrayList<ArrayList<PathPoint>> sections = new ArrayList<>();

        //pathPoint init here
        ArrayList<PathPoint> start = new ArrayList<>();
        start.add(new PathPoint(138.34, 21.6, -1, 1, 12, 0));
        start.add(new PathPoint(138.34, 21.6, -1, 1, 12, 0));
        start.add(new PathPoint(80, 80, -1, 1, 12, 0));
        start.add(new PathPoint(100, 170, -0.5, 1, 12, 0));

        ArrayList<PathPoint> section1 = new ArrayList<>();
        section1.add(new PathPoint(210, 200, -1, 1, 15, 0));

        sections.add(start);

        final PurePursuit controller = new PurePursuit(sections);

        super.controller = controller;

        waitForStart();

        Robot.startOdometryThread();
        startInterruptHandler();

        new Thread(new Runnable() {
            @Override
            public void run() {
                //potential sleep here
                if(!isStopRequested()) {
                    controller.run();
                }
            }
        }).start();

        //im stupid,,, just remove the while(opModeIsActive()) and everything is better
        while(controller.getCurrentSection() == 0 && !isStopRequested()) {
            subsystems.shooter.setPowers(0.895);
            subsystems.shooter.setPowerShotPosition(0.05);
            subsystems.wobbleGrabber.wobbleGrab.setPosition(1.5);
            subsystems.wobbleGrabber.wobbleTurn.setPosition(0.52);
            //any point specific actions can start here

        }
        subsystems.shooter.setPowers(0.895);
        subsystems.drivetrain.resetAiming();
        while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
            subsystems.shooter.setHopperPosition(0.0);
            subsystems.shooter.setPowers(0.895);
            subsystems.drivetrain.aim(1, 0.005);
        }
        subsystems.drivetrain.run(0, 0);
        sleep(100);
        subsystems.shooter.setHopperPosition(0.15);
        sleep(500);
        subsystems.drivetrain.resetAiming();
        while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
            subsystems.shooter.setHopperPosition(0.0);
            subsystems.shooter.setPowers(0.895);
            subsystems.drivetrain.aim(2, 0.01);
        }
        subsystems.drivetrain.run(0, 0);
        sleep(100);
        subsystems.shooter.setHopperPosition(0.15);
        sleep(500);
        subsystems.drivetrain.resetAiming();
        while(!subsystems.drivetrain.isAimed() && !isStopRequested()) {
            subsystems.shooter.setHopperPosition(0.0);
            subsystems.shooter.setPowers(0.895);
            subsystems.drivetrain.aim(3, 0.01);
        }
        subsystems.drivetrain.run(0, 0);
        sleep(100);
        subsystems.shooter.setHopperPosition(0.15);
        sleep(500);
        subsystems.drivetrain.resetAiming();
        subsystems.shooter.setHopperPosition(0.0);
        subsystems.shooter.setPowers(0.0);
        controller.startNextSection();
        //between while loops is where we call startNextSection() and everything works out great omg
        while(controller.getCurrentSection() == 1 && !isStopRequested()) {
            subsystems.wobbleGrabber.wobbleTurn.setPosition(0.52);
        }

        Robot.storeRobotPosition();

    }
}
