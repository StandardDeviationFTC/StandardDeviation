package org.firstinspires.ftc.teamcode.lib.misc;

public class Constants {

    public static final float MM_PER_INCH = 25.4f;
    public static final float TARGET_HEIGHT_MM = 6.375f * MM_PER_INCH; // the height of the center of the target image above the floor

    public static final float HALF_FIELD_INCHES = 72;
    public static final float HALF_TILE_INCHES = 12;
    public static final float ONE_TILE_INCHES = 24;
    public static final float ONE_AND_HALF_TILE_INCHES = 36;

    public static final float HALF_FIELD_MM = HALF_FIELD_INCHES * MM_PER_INCH;
    public static final float HALF_TILE_MM = HALF_TILE_INCHES * MM_PER_INCH;
    public static final float ONE_TILE_MM = ONE_TILE_INCHES * MM_PER_INCH;
    public static final float ONE_AND_HALF_TILE_MM = ONE_AND_HALF_TILE_INCHES * MM_PER_INCH;

    public static final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * MM_PER_INCH;
    public static final float CAMERA_VERTICAL_DISPLACEMENT = 3.7f * MM_PER_INCH;
    public static final float CAMERA_LEFT_DISPLACEMENT = 0.0f * MM_PER_INCH;

    private static final float ENCODER_COUNTS_PER_REVOLUTION = 540;
    private static final float WHEEL_DIAMETER_INCHES = 4;
    private static final float INCHES_PER_REVOLUTION = (float) (WHEEL_DIAMETER_INCHES * Math.PI);

    public static final float ENCODER_COUNTS_PER_INCH = ENCODER_COUNTS_PER_REVOLUTION / INCHES_PER_REVOLUTION;

    public static final float AUTO_TURN_HEADING_TOLERANCE_DEG_PER_SECOND = 0.1f;
    public static final float AUTO_DRIVE_DISTANCE_TOLERANCE_COUNTS_PER_SECOND = 0.5f;

    public static final float AUTO_TURN_HEADING_TOLERANCE_DEG = 1;
    public static final float AUTO_DRIVE_DISTANCE_TOLERANCE_COUNTS = 0.5f * ENCODER_COUNTS_PER_INCH;

}
