package org.firstinspires.ftc.teamcode.lib.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.lib.autonomous.VuforiaNavigation;

public class Bot {

    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private float frontLeftEncoderStart;
    private float frontRightEncoderStart;
    private float backLeftEncoderStart;
    private float backRightEncoderStart;

    private BNO055IMU imu;
    private VuforiaNavigation vuforiaNavigation;

    public Bot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.vuforiaNavigation = new VuforiaNavigation(this);
        this.telemetry = opMode.telemetry;
        initMotors();
        initIMU();
    }

    public boolean opModeIsActive() {
        return opMode.opModeIsActive();
    }

    public Telemetry getTelemetry() {
        return this.telemetry;
    }

    public void cleanUp() {
        vuforiaNavigation.cleanUp();
    }

    public void update() {
        telemetry.update();
        vuforiaNavigation.update();
    }

    public VuforiaNavigation getVuforiaNavigation() {
        return this.vuforiaNavigation;
    }

    private void initMotors() {
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");

        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public Orientation getOrientationDeg(AxesOrder order) {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, order, AngleUnit.DEGREES);
    }

    public boolean isCalibrated() {
        return imu.isGyroCalibrated();
    }

    public DcMotor getFrontRightDrive() {
        return this.frontRightDrive;
    }

    public DcMotor getFrontLeftDrive() {
        return this.frontLeftDrive;
    }

    public DcMotor getBackRightDrive() {
        return this.backRightDrive;
    }

    public DcMotor getBackLeftDrive() {
        return this.backRightDrive;
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
        frontLeftDrive.setPower(power);
    }

    public void setFrontRightDrivePower(double power) {
        frontRightDrive.setPower(power);
    }

    public void setBackLeftDrivePower(double power) {
        backLeftDrive.setPower(power);
    }

    public void setBackRightDrivePower(double power) {
        backRightDrive.setPower(power);
    }

    public void setLeftDrivePower(double power) {
        backLeftDrive.setPower(power);
        frontLeftDrive.setPower(power);
    }

    public void setRightDrivePower(double power) {
        backRightDrive.setPower(power);
        frontRightDrive.setPower(power);
    }

}
