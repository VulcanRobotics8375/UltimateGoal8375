package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.vulcanrobotics.robotcorelib.framework.AutoPipeline;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.math.PathPoint;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.PurePursuit;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import java.util.ArrayList;

//@Disabled
@Autonomous(name = "example path", group = "example")
public class ExamplePath extends AutoPipeline {

    volatile boolean doneShooting = false;

    @Override
    public void runOpMode() {
        try {
            //backend initialization
            autoInit();
            //set starting coefficients, all board measurements in CM this year
            setStart(new Point(0, 0), 0);
            //defining the data structure for path points
            ArrayList<ArrayList<PathPoint>> sections = new ArrayList<>();
            ArrayList<PathPoint> section1 = new ArrayList<>();
            section1.add(new PathPoint(0, 0, -0.5, 0, 1, 0));
            section1.add(new PathPoint(30, 40, -0.5, 0, 1, 0));
            sections.add(section1);

            //path points here

            //initialize the Pipeline Controller with whatever controller we are using
            controller = new PurePursuit(sections);

           final PurePursuit internalController = (PurePursuit) controller;

           telemetry.addLine("ready");
           telemetry.update();

            waitForStart();

            //interrupt handler makes sure the robot stops when the stop button is pressed.
            Robot.startOdometryThread();
            startInterruptHandler();

            new Thread(new Runnable() {
                @Override
                public void run() {

                }
            }).start();

            while(opModeIsActive()) {

                PathPoint currentPoint = internalController.getCurrentPoint();
                //subsystem code here, triggers are currentPoint and then timing
                internalController.moveToPoint(section1.get(1));

                telemetry.addData("robot x", Robot.getRobotX());
                telemetry.addData("robot y", Robot.getRobotY());
                telemetry.addData("robot angle", Robot.getRobotAngleDeg());

                telemetry.update();

                if(isStopRequested())
                    break;
            }

            controller.stop = true;

        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }




    }
}
