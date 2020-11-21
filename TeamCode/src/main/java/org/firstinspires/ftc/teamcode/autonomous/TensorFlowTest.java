package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.vulcanrobotics.robotcorelib.framework.AutoPipeline;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;
import org.vulcanrobotics.robotcorelib.robot.Robot;

import java.util.List;

@Autonomous(name = "stack detector TFOD test", group = "test")
public class TensorFlowTest extends AutoPipeline {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "Aazej9z/////AAABmYTWVbTlO04wi/NEEmgCikhNhTfUHtKzh3qbkkR8LfNtAliRi6J7R2C+YyPyA3z4A9wWiTSPQTc2mb/W+UHjAQ9V/ggozzYUxDjnarBz0PoPvjfJevEZJ0t3XCGvDjYIU0pKVwIFF6T6rqPGxTM+H9oDCMaeu6R6dDLtyS+YzIwLkgCAkwGa+vnbRWFbnB1OgYvYodvW+DkBKxHkwEElJAGLjMf1mLJb+zjCuU9IAOirXQGlhmpxNcLGhHQxpxLi21p8cBbBsoJXGhmBsZ0KnK1+SwEdcTIDqkT2XQlrCE/RjS1FNj97EM4wTps6s+XDWG/lce+K+DRpy91jdIHqGnJIyeCHsQm9LnHieKbuDt5N";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        try {

            autoInit();
            initVuforia();
            initTfod();

            if(tfod != null) {
                tfod.activate();

//                tfod.setZoom(1.0, 16.0/9.0);
            }

            waitForStart();

            Robot.startOdometryThread();
            startInterruptHandler();

            while(opModeIsActive()) {
                if(tfod != null) {
                    List<Recognition> recognitions = tfod.getUpdatedRecognitions();
                    if(recognitions != null) {
                        telemetry.addData("# Object Detected", recognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : recognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();

                    }
                }
            }

        } catch (RobotCoreLibException e) {
            e.printStackTrace();
        }


    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
