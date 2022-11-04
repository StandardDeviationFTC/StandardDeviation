package org.firstinspires.ftc.teamcode.lib.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.lib.autonomous.core.ReferenceFrame;
import org.firstinspires.ftc.teamcode.lib.autonomous.core.VuforiaNavigation;
import org.firstinspires.ftc.teamcode.lib.misc.Constants;

/**
Contains all of the robot's hardware devices and other utilities including:
<ul>
<li>Drive and other motors</li>
<li>Servos</li>
<li>Control Hub IMU</li>
<li>Vuforia Navigation</li>
</ul>
 */

public class Bot {

    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private DcMotor frontLeftDrive, frontRightDrive;
    private DcMotor backLeftDrive, backRightDrive;
    private DcMotorSimple lift;

    private DigitalChannel liftLimitSwitch;

    private Servo leftClaw, rightClaw;
    private Servo clawFlipper;

    private float frontLeftEncoderStart, frontRightEncoderStart;
    private float backLeftEncoderStart, backRightEncoderStart;

    private BNO055IMU imu;
    private VuforiaNavigation vuforiaNavigation;

    /**
     * Creates a new Robot object and initializes hardware devices and utilities
     *
     * @param opMode the current opMode that will run the robot
     */
    public Bot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        //this.vuforiaNavigation = new VuforiaNavigation(this);
        this.telemetry = opMode.telemetry;

        initHardware();
        initIMU();
    }

    /**
     * Sets the reference frame to be used when doing vuforia calculations to determine the robot's location
     *
     * @param referenceFrame the frame of reference from which field positions should be measured
     */
    public void setReferenceFrame(ReferenceFrame referenceFrame) {
        vuforiaNavigation.setReferenceFrame(referenceFrame);
    }

    /**
     * @return true if the current opmode is active
     */
    public boolean opModeIsActive() {
        return opMode.opModeIsActive();
    }

    /**
     * @return the Telemetry object for communication with the Driver Hub
     */
    public Telemetry getTelemetry() {
        return this.telemetry;
    }

    /**
     * Deactivates vuforia navigation targets
     */
    public void cleanUp() {
        vuforiaNavigation.cleanUp();
    }

    /**
     * Sends telemetry to the Driver Hub and polls a new location from VuforiaNavigation
     */
    public void update() {
        telemetry.update();
        //vuforiaNavigation.updateLocation();
    }

    /**
     * @return the VuforiaNavigation object tied to this instance
     */
    public VuforiaNavigation getVuforiaNavigation() {
        return this.vuforiaNavigation;
    }

    private void initHardware() {
        liftLimitSwitch = hardwareMap.get(DigitalChannel.class, "liftLimitSwitch");

        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");

        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = hardwareMap.get(DcMotorSimple.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        clawFlipper = hardwareMap.get(Servo.class, "clawFlipper");
    }

    private void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public HardwareMap getHardwareMap() {
        return this.hardwareMap;
    }

    public float getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public boolean getLiftLimitSwitchState() {
        return liftLimitSwitch.getState();
    }

    public Orientation getOrientation(AxesOrder order) {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, order, AngleUnit.DEGREES);
    }

    public boolean isCalibrated() {
        return imu.isGyroCalibrated();
    }

    public void setLiftPower(float power) {
        this.lift.setPower(power);
    }

    public void zeroServos() {
        clawFlipper.setPosition(0);
        leftClaw.setPosition(0);
        rightClaw.setPosition(Constants.FLIPPER_ACTUATE_DISTANCE);
    }

    public void flipClaw() {
        if(clawFlipper.getPosition() == 0)
            clawFlipper.setPosition(Constants.FLIPPER_ACTUATE_DISTANCE);
        else
            clawFlipper.setPosition(0);
    }

    public void setFlipperPosition(double position) {
        clawFlipper.setPosition(position);
    }

    public void toggleClaw() {
        if(rightClaw.getPosition() == 0) {
            leftClaw.setPosition(0);
            rightClaw.setPosition(Constants.CLAW_ACTUATE_DISTANCE);
        } else {
            leftClaw.setPosition(Constants.CLAW_ACTUATE_DISTANCE);
            rightClaw.setPosition(0);
        }
    }

    public void openClaw() {
        leftClaw.setPosition(Constants.LEFT_CLAW_CLOSED_POSITION + Constants.CLAW_ACTUATE_DISTANCE);
        rightClaw.setPosition(Constants.RIGHT_CLAW_CLOSED_POSITION - Constants.CLAW_ACTUATE_DISTANCE);
    }

    public void closeClaw() {
        leftClaw.setPosition(Constants.LEFT_CLAW_CLOSED_POSITION - Constants.CLAW_CLOSED_OFFSET);
        rightClaw.setPosition(Constants.RIGHT_CLAW_CLOSED_POSITION + Constants.CLAW_CLOSED_OFFSET);
    }

    public void setFlipperPosition(float position) {
        clawFlipper.setPosition(position);
    }

    public void setClawPositions(float left, float right) {
        leftClaw.setPosition(left);
        rightClaw.setPosition(right);
    }

    public float getFrontRightEncoder() {
        return this.frontRightDrive.getCurrentPosition() - this.frontRightEncoderStart;
    }

    public float getFrontLeftEncoder() {
        return this.frontLeftDrive.getCurrentPosition() - this.frontLeftEncoderStart;
    }

    public float getBackRightEncoder() {
        return this.backRightDrive.getCurrentPosition() - this.backRightEncoderStart;
    }

    public float getBackLeftEncoder() {
        return this.backLeftDrive.getCurrentPosition() - this.backLeftEncoderStart;
    }

    public void resetEncoders() {
        this.frontRightEncoderStart = frontRightDrive.getCurrentPosition();
        this.frontLeftEncoderStart = frontLeftDrive.getCurrentPosition();
        this.backLeftEncoderStart = backLeftDrive.getCurrentPosition();
        this.backRightEncoderStart = backRightDrive.getCurrentPosition();
    }

    public void setFrontLeftDrivePower(double power) {
        frontLeftDrive.setPower(Constants.MAX_MOTOR_SPEED * power);
    }

    public void setFrontRightDrivePower(double power) {
        frontRightDrive.setPower(Constants.MAX_MOTOR_SPEED * power);
    }

    public void setBackLeftDrivePower(double power) {
        backLeftDrive.setPower(Constants.MAX_MOTOR_SPEED * power);
    }

    public void setBackRightDrivePower(double power) {
        backRightDrive.setPower(Constants.MAX_MOTOR_SPEED * power);
    }

    public void setLeftDrivePower(double power) {
        setBackLeftDrivePower(power);
        setFrontLeftDrivePower(power);
    }

    public void setRightDrivePower(double power) {
        setBackRightDrivePower(power);
        setFrontRightDrivePower(power);
    }

}
