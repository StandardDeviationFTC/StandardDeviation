package org.firstinspires.ftc.teamcode.lib.autonomous;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.lib.robot.Bot;
import org.firstinspires.ftc.teamcode.lib.misc.Constants;
import org.firstinspires.ftc.teamcode.lib.robot.MeccanumDrive;
import org.firstinspires.ftc.teamcode.lib.pid.PIDAngleInput;
import org.firstinspires.ftc.teamcode.lib.pid.PIDController;
import org.firstinspires.ftc.teamcode.lib.pid.PIDLinearInput;
import org.firstinspires.ftc.teamcode.lib.misc.Util;

public class AutonomousMeccanumDrive extends MeccanumDrive {

    private PIDController headingController;
    private PIDController distanceController;

    private VuforiaNavigation vuforiaNavigation;

    private PIDAngleInput targetHeading;
    private PIDAngleInput currentHeading;

    private PIDLinearInput targetDistance;
    private PIDLinearInput currentDistance;

    public AutonomousMeccanumDrive(Bot robot) {
        super(robot);

        this.vuforiaNavigation = robot.getVuforiaNavigation();
        this.headingController = new PIDController(-0.016, 0.2, 0.3);
        this.distanceController = new PIDController(-0.002, 0.025, 0.4);

        this.targetHeading = new PIDAngleInput(0);
        this.currentHeading = new PIDAngleInput(0);

        this.headingController.setTarget(targetHeading);
        this.headingController.setInput(currentHeading);

        this.targetDistance = new PIDLinearInput(0);
        this.currentDistance = new PIDLinearInput(0);

        this.distanceController.setTarget(targetDistance);
        this.distanceController.setInput(currentDistance);
    }

    public void setDriveDistanceInches(float inches) {
        this.distanceController.reset();
        this.currentDistance.value = 0;
        this.targetDistance.value = Constants.ENCODER_COUNTS_PER_INCH * inches;
        this.robot.resetEncoders();
    }

    public void setTargetHeading(float heading) {
        this.headingController.reset();
        this.targetHeading.setAngle(heading);
    }

    public void update() {
        currentHeading.setAngle(robot.getOrientationDeg(AxesOrder.XYZ).thirdAngle);
        currentDistance.value = getAverageEncoderValues();

        float turn = Range.clip((float) headingController.getResponse(), -1.0f, 1.0f);
        float headingDrive = Range.clip((float) distanceController.getResponse(), -1.0f + turn/2, 1.0f - turn/2);

        double tgtHeadingRadians = Math.toRadians(targetHeading.getAngle());
        float xDrive = (float) Math.cos(tgtHeadingRadians) * headingDrive;
        float yDrive = (float) -Math.sin(tgtHeadingRadians) * headingDrive;

        VectorF driveStrafeVector = Util.getDriveStrafeVector(xDrive, yDrive, (float) currentHeading.getAngle());

        float drive = driveStrafeVector.get(0);
        float strafe = driveStrafeVector.get(1);

        robot.getTelemetry().addData("Drive", drive);
        robot.getTelemetry().addData("Turn", turn);
        robot.getTelemetry().addData("Strafe", strafe);


        super.setDrive(drive, turn, strafe);
        super.applyPower();

        robot.update();
    }

    public float getAverageEncoderValues() {
        float sum = 0;

        sum += robot.getFrontLeftEncoder();
        sum += robot.getFrontRightEncoder();
        sum += robot.getBackLeftEncoder();
        sum += robot.getBackRightEncoder();

        return sum / 4;
    }

}
