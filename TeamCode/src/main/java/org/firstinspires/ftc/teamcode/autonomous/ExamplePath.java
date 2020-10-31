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

@Disabled
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

            //path points here

            //initialize the Pipeline Controller with whatever controller we are using
            controller = new PurePursuit(sections);

            waitForStart();

            //interrupt handler makes sure the robot stops when the stop button is pressed.
            startInterruptHandler();

            new Thread(new Runnable() {
                @Override
                public void run() {
                    while(!doneShooting) {

                    }
                    controller.run();
                }
            }).start();

            while(opModeIsActive()) {

                PathPoint currentPoint = controller.getCurrentPoint();
                //subsystem code here, triggers are currentPoint and then timing


                if(isStopRequested())
                    break;
            }

        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }




    }
}
