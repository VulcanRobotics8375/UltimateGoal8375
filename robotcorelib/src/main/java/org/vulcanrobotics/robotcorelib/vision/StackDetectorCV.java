package org.vulcanrobotics.robotcorelib.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.vulcanrobotics.robotcorelib.vision.analysis.AverageBrightnessAnalysis;

import java.util.ArrayList;
import java.util.List;

public class StackDetectorCV extends OpenCvPipeline {

    Mat ringColorFilter = new Mat();
    Mat workingMat = new Mat();
    Mat heirarchy = new Mat();
    private double stackContourHeight;
    private int stackHeight;

    AverageBrightnessAnalysis brightnessAnalysis = new AverageBrightnessAnalysis();

    //TODO Add brightness adjustments
    //TODO add multi-frame scoring
    //TODO update HSV and contourHeight constants

    @Override
    public Mat processFrame(Mat input) {
        Mat output = input.clone();
        Imgproc.blur(input, workingMat, new Size(5, 5));
        Imgproc.cvtColor(workingMat, workingMat, Imgproc.COLOR_BGR2HSV);

//        brightnessAnalysis.run(input);

        Imgproc.GaussianBlur(workingMat, workingMat, new Size(5,5), 0);

        Core.inRange(workingMat, new Scalar(53, 130, 200), new Scalar(150, 255, 255), ringColorFilter);

        List<MatOfPoint> ringContours = new ArrayList<>();

        Imgproc.findContours(ringColorFilter, ringContours, heirarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

//        Imgproc.drawContours(output, ringContours, -1, new Scalar(255,0,0), 2);

        double biggestContour = Double.MIN_VALUE;
        Rect boundingRect = new Rect();

        for (MatOfPoint contour :
                ringContours) {
            if(contour.size().height > biggestContour) {
                biggestContour = contour.size().height;
                boundingRect = Imgproc.boundingRect(contour);
            }
        }

        Imgproc.rectangle(output, boundingRect, new Scalar(255, 0, 0));
        Imgproc.putText(output, "detectedStack", new Point(boundingRect.x, boundingRect.y), Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar(255, 0, 0));

        stackContourHeight = biggestContour;

        if(stackContourHeight * 2 < 290) {
            stackHeight = 1;
        }
        if(stackContourHeight * 2 > 290 && stackContourHeight * 2 < 350) {
            stackHeight = 2;
        }
        if(stackContourHeight * 2 > 350) {
            stackHeight = 3;
        }

        ringColorFilter.release();
        workingMat.release();
//        input.release();
//        output.release();

        return output;

    }

    public int getStackHeight() {
        return stackHeight;
    }

    public double getStackContourHeight() {
        return stackContourHeight;
    }

}
