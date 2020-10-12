package org.vulcanrobotics.robotcorelib.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.vulcanrobotics.robotcorelib.framework.AutoPipeline;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;

@Autonomous(name = "vision test", group = "vision")
public class VisionExample extends AutoPipeline {

    StackDetectorCV stackDetector = new StackDetectorCV();

    @Override
    public void runOpMode() {
        initVision(stackDetector);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("testing", stackDetector.getStackHeight());
            telemetry.addData("contour", stackDetector.getStackContourHeight());
            telemetry.update();
        }
    }
}
