package org.vulcanrobotics.robotcorelib.framework;

public class Constants {

    /**
     * List all constants here.
     */

    public static final String EXAMPLE = "Hello, world!";
    public static final double FIELD_SIZE_CM_Y = 365.76;
    public static final double FIELD_SIZE_CM_X = 243.84;
    public static final double TILE_SIZE_CM = 60.96;
    public static final double ODOMETRY_TICKS_PER_REV = 1440;
    public static final double ODOMETRY_WHEELBASE = 37.85; // needs recalibration, theoretical value is 36.42
    public static final double ODOMETRY_RADIUS = 1.9;
    public static final double ODOMETRY_COUNTS_PER_CM = 1440.0 / (2.0 * Math.PI * ODOMETRY_RADIUS);
    public static final double ODOMETRY_HORIZONTAL_TICKS_PER_REV = 19.3;
    public static final double SHOOTING_OFFSET_RAD = 14.0 * (Math.PI / 180.0);


}
