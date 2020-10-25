package org.vulcanrobotics.robotcorelib.vision.filters;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;

public class HSVColorFilter extends Filter {

    private Scalar hsv;

    public HSVColorFilter(Scalar hsv) {
        this.hsv = hsv;
    }

    @Override
    public Mat process(Mat input) {
        return null;
    }

    public Scalar getHsvValues() {
        return hsv;
    }
}
