package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.vulcanrobotics.robotcorelib.framework.AutoPipeline;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;

@Autonomous(name = "odometry calibration", group = "calibration")
public class OdometryCalibration extends AutoPipeline {

    @Override
    public void runOpMode() {
        try {
            autoInit();
            subsystems.drivetrain.initIMU();
            subsystems.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();

        subsystems.motionProfile.calibrate(-0.5);

        while(opModeIsActive()) {}

    }
}
