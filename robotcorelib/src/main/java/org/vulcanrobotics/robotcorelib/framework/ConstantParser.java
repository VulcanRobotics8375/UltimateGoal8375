package org.vulcanrobotics.robotcorelib.framework;

import org.vulcanrobotics.robotcorelib.robot.Robot;

public class ConstantParser {

    public static double parseDouble(String val) {
        return Double.parseDouble(Robot.getConstants().getProperty(val));
    }

    public static int parseInt(String val) {
        return Integer.parseInt(Robot.getConstants().getProperty(val));
    }

    public static float parseFloat(String val) {
        return Float.parseFloat(Robot.getConstants().getProperty(val));
    }

    public static boolean parseBoolean(String val) {
        return Boolean.parseBoolean(Robot.getConstants().getProperty(val));
    }

}
