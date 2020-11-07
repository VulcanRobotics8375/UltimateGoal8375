package org.vulcanrobotics.robotcorelib.framework;

public class Constants {

    /**
     * List all constants here.
     */

    public static final String EXAMPLE = "Hello, world!";
    public static final double DRIVETRAIN_SLOW_MODE_MULTIPLIER = 0.5;
    public static final double FIELD_SIZE_CM_Y = 365.76;
    public static final double FIELD_SIZE_CM_X = 243.84;
    public static final double TILE_SIZE_CM = 60.96;
    public static final double SHOOTER_AUTO_ALIGN_GAIN = 1;
    public static final double SHOOTER_ANGLE_OFFSET_PER_IN = 0;
    public static final double INTAKE_LEFT_POWER = 1;
    public static final double INTAKE_RIGHT_POWER = 1;
    public static final double ODOMETRY_TICKS_PER_REV = 1440;
    public static final double ODOMETRY_WHEELBASE = 37.85; // needs recalibration, theoretical value is 36.42
    public static final double ODOMETRY_RADIUS = 1.9;
    public static final double ODOMETRY_COUNTS_PER_CM = 1440.0 / (2.0 * Math.PI * ODOMETRY_RADIUS);
    public static final double ODOMETRY_HORIZONTAL_TICKS_PER_REV = 18.3;
    public static final double SHOOTING_DEGREE_BIAS = 90;
    public static final double SHOOTING_OFFSET_MAX = 12;
    public static final double SHOOTING_OFFSET_MIN = 8;



}
