package org.firstinspires.ftc.teamcode.lib.autonomous.core;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.lib.robot.Bot;
import org.firstinspires.ftc.teamcode.lib.misc.Constants;
import org.firstinspires.ftc.teamcode.lib.robot.MeccanumDrive;
import org.firstinspires.ftc.teamcode.lib.pid.PIDAngleInput;
import org.firstinspires.ftc.teamcode.lib.pid.PIDController;
import org.firstinspires.ftc.teamcode.lib.pid.PIDLinearInput;
import org.firstinspires.ftc.teamcode.lib.misc.Util;

public class AutonomousMeccanumDrive extends MeccanumDrive {

    private PIDController headingController;
    private PIDController positionController;

    private VuforiaNavigation vuforiaNavigation;

    private PIDAngleInput targetHeading;
    private PIDAngleInput currentHeading;

    private PIDLinearInput currentPosition;
    private PIDLinearInput targetPosition;

    private float fieldX;
    private float fieldY;

    private float vuforiaFieldX;
    private float vuforiaFieldY;

    private AutonomousInstruction currentInstruction;
    private DriveMode mode;

    public AutonomousMeccanumDrive(Bot robot) {
        super(robot, 100000);

        this.vuforiaNavigation = robot.getVuforiaNavigation();
        this.headingController = new PIDController(-0.016, 0.2, 0.3);
        this.positionController = new PIDController(-0.002, 0.025, 0.4);

        this.targetHeading = new PIDAngleInput(0);
        this.currentHeading = new PIDAngleInput(0);

        this.headingController.setTarget(targetHeading);
        this.headingController.setInput(currentHeading);

        this.currentPosition = new PIDLinearInput(0);
        this.targetPosition = new PIDLinearInput(0);

        this.positionController.setTarget(targetPosition);
        this.positionController.setInput(currentPosition);
    }

    public void setInstruction(AutonomousInstruction instruction) {
        this.currentInstruction = instruction;
    }

    public void driveDistance(DriveMode mode, float inches) {
        this.positionController.reset();
        this.targetPosition.value = currentPosition.value + Constants.ENCODER_COUNTS_PER_INCH * inches * mode.getDirection();
        this.robot.resetEncoders();
        this.mode = mode;

        updatePositionPID();
    }

    private void updatePositionPID() {
        if(mode.isStrafe()) {
            this.currentPosition.value = fieldX;
        } else {
            this.currentPosition.value = fieldY;
        }
    }

    public void setTargetHeading(float heading) {
        this.headingController.reset();
        this.targetHeading.setAngle(heading);
    }

    public void update() {
        updateDrive();
        updateLocation();
    }

    private void updateLocation() {
        if(vuforiaNavigation.isTracking()) {
            VectorF position = vuforiaNavigation.getPosition();
            this.vuforiaFieldX = position.get(0);
            this.vuforiaFieldY = position.get(1);
            this.fieldX = vuforiaFieldX;
            this.fieldY = vuforiaFieldY;

            robot.resetEncoders();
        } else {
            if(mode.isDrive()) {
                this.fieldY = vuforiaFieldY + getDrivenDistance();
            } else {
                this.fieldX = vuforiaFieldX + getDrivenDistance();
            }
        }

        updatePositionPID();
    }

    public float getFieldX() {
        return this.fieldX;
    }

    public float getFieldY() {
        return this.fieldY;
    }

    public float getHeadingError() {
        return (float) headingController.getCurrentError();
    }

    public float getPositionError() {
        return (float) positionController.getCurrentError();
    }

    private void updateDrive() {
        currentHeading.setAngle(robot.getHeading());

        float turn = Range.clip((float) headingController.getResponse(), -1.0f, 1.0f);
        float positionResponse = Range.clip((float) positionController.getResponse(), -1.0f + turn/2, 1.0f - turn/2);

        float xDrive = mode.isStrafe() ? positionResponse : 0;
        float yDrive = mode.isDrive() ? positionResponse : 0;

        VectorF driveStrafeVector = Util.getDriveStrafeVector(xDrive, yDrive, (float) currentHeading.getAngle());

        float drive = driveStrafeVector.get(0);
        float strafe = driveStrafeVector.get(1);

        robot.getTelemetry().addData("Drive", drive);
        robot.getTelemetry().addData("Turn", turn);
        robot.getTelemetry().addData("Strafe", strafe);

        robot.getTelemetry().addData("Field X", fieldX);
        robot.getTelemetry().addData("Field Y", fieldY);

        super.setDrive(drive, turn, strafe);
        super.applyPower();

        robot.update();
    }

    public float getDrivenDistance() {
        if(mode == DriveMode.FORWARDS || mode == DriveMode.BACKWARDS) {
            float driveSum = 0;

            driveSum += robot.getFrontLeftEncoder();
            driveSum += robot.getFrontRightEncoder();
            driveSum += robot.getBackLeftEncoder();
            driveSum += robot.getBackRightEncoder();

            driveSum /= 4;
            return (mode == DriveMode.FORWARDS) ? driveSum : -driveSum;
        } else {
            float strafeSum = 0;

            strafeSum += robot.getFrontLeftEncoder();
            strafeSum += robot.getFrontRightEncoder();
            strafeSum -= robot.getBackLeftEncoder();
            strafeSum -= robot.getBackRightEncoder();

            strafeSum /= 4;
            return (mode == DriveMode.RIGHT) ? strafeSum : -strafeSum;
        }
    }

}
