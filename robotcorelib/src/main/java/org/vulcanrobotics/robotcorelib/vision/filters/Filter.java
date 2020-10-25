package org.vulcanrobotics.robotcorelib.vision.filters;

import org.opencv.core.Mat;

public abstract class Filter {

    public abstract Mat process(Mat input);

}
