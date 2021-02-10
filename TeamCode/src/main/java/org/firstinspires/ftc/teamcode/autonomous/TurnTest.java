package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.vulcanrobotics.robotcorelib.framework.AutoPipeline;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.math.PathPoint;
import org.vulcanrobotics.robotcorelib.motion.PurePursuit;

import java.util.ArrayList;

@Autonomous(name = "turn test", group = "test")
public class TurnTest extends AutoPipeline {
    @Override
    public void runOpMode() {
        try {
            autoInit();
        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }

        ArrayList<ArrayList<PathPoint>> sections = new ArrayList<>();

        ArrayList<PathPoint> test = new ArrayList<>();
        test.add(new PathPoint(140, 60, 0, 1, 15, 0));

        sections.add(test);

        PurePursuit controller = new PurePursuit(sections);

        waitForStart();

        while(opModeIsActive()) {
            controller.run();
        }


    }
}
