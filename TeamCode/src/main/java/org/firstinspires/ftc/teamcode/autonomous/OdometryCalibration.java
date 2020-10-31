package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.vulcanrobotics.robotcorelib.framework.AutoPipeline;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;

@Autonomous(name = "odometry calibration", group = "calibration")
public class OdometryCalibration extends AutoPipeline {

    @Override
    public void runOpMode() {
        try {
            autoInit();
        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();

//        while(opModeIsActive()) {}

        subsytems.motionProfile.calibrate(0.2);

        while(opModeIsActive()) {}

    }
}
