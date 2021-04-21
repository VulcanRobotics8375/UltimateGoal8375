package org.vulcanrobotics.robotcorelib.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    protected RobotConfig subsystems;
    private OpenCvCamera camera;

    public abstract void runOpMode();

    public void autoInit() throws RobotCoreLibException {
        Robot.setTelemetry(telemetry);
        Robot.hardwareMap = hardwareMap;
        Robot.init();
        subsystems = Robot.getComponents();
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

    public void initVision(OpenCvPipeline pipeline, boolean webcam) {
//        final OpenCvCamera camera;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        if(webcam) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        } else {
            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

//        camera.openCameraDevice();
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });
        camera.setPipeline(pipeline);


    }

    public void stopVision() {
        camera.stopStreaming();
//        camera.closeCameraDevice();
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
               controller.stop = true;
            }
        }).start();
    }

}
