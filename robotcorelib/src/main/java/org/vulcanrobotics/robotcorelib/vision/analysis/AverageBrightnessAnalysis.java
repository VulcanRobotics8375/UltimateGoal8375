package org.vulcanrobotics.robotcorelib.vision.analysis;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class AverageBrightnessAnalysis extends VisionAnalysis {

    private double avgBrightness;

    Mat grayscale = new Mat();

    @Override
    public Mat run(Mat input) {

        List<Double> values = new ArrayList<>();

        Imgproc.cvtColor(input, grayscale, Imgproc.COLOR_BGR2GRAY);
        for (int i = 0; i < grayscale.rows(); i++) {
            for (int j = 0; j < grayscale.cols(); j++) {
                double value = grayscale.get(i, j)[0];
                values.add(value);
            }
        }

        double sum = 0;
        for (double value : values) {
            sum += value;
        }
        avgBrightness = sum / values.size();

        return input;

    }

    public double getAvgBrightness() {
        return avgBrightness;
    }
}
