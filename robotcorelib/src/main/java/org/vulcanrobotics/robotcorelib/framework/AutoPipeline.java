package org.vulcanrobotics.robotcorelib.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.vulcanrobotics.robotcorelib.math.Point;
import org.vulcanrobotics.robotcorelib.motion.Controller;
import org.vulcanrobotics.robotcorelib.robot.Robot;

public abstract class AutoPipeline extends LinearOpMode {

    protected Controller controller;

    public abstract void runOpMode();

    public void autoInit() throws RobotCoreLibException {
        Robot.setTelemetry(telemetry);
        Robot.init();
    }

    public void setStart(Point position, double angle) {
        Robot.setRobotPos(position);
        Robot.setRobotAngle(angle);
    }

    public void initVision(OpenCvPipeline pipeline) {
        OpenCvCamera phoneCamera;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCamera.openCameraDevice();
        phoneCamera.setPipeline(pipeline);
        phoneCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

    }

    public void initVision(OpenCvPipeline pipeline, OpenCvInternalCamera.CameraDirection direction, OpenCvCameraRotation rotation, Point resolution) {
        OpenCvCamera phoneCamera;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(direction, cameraMonitorViewId);

        phoneCamera.openCameraDevice();
        phoneCamera.setPipeline(pipeline);
        phoneCamera.startStreaming((int)resolution.x, (int)resolution.y, rotation);
    }

    public void startInterruptHandler() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                while(!isStopRequested()) {}
               controller.stop = isStopRequested();
            }
        }).start();
    }

}
