package org.firstinspires.ftc.teamcode.lib.autonomous;

import org.firstinspires.ftc.teamcode.lib.misc.Constants;
import org.firstinspires.ftc.teamcode.lib.misc.Stopwatch;

public class AutonomousDriveInstruction extends AutonomousInstruction {

    private int targetCounts;
    private float targetInches;
    private Stopwatch stopwatch;

    public AutonomousDriveInstruction(float inches) {
        this.targetInches = inches;
        this.targetCounts = (int) (Constants.ENCODER_COUNTS_PER_INCH * inches);
        this.stopwatch = new Stopwatch();
    }

    @Override
    public void execute() {
        float startEncoderValue = super.drive.getAverageEncoderValues();
        super.drive.setDriveDistanceInches(targetInches);

        stopwatch.restart();
        float lastEncoderValue = startEncoderValue;

        while(super.robot.opModeIsActive()) {
            super.drive.update();

            float encoderValue = super.drive.getAverageEncoderValues();
            float speed = (encoderValue - lastEncoderValue) / stopwatch.getSeconds();
            float countsTraveled = encoderValue - startEncoderValue;

            if(Math.abs(countsTraveled - targetCounts) < Constants.AUTO_DRIVE_DISTANCE_TOLERANCE_COUNTS &&
                    speed < Constants.AUTO_DRIVE_DISTANCE_TOLERANCE_COUNTS_PER_SECOND)
                break;

            super.robot.getTelemetry().addData("Speed", speed);
            super.robot.getTelemetry().addData("Counts Traveled", countsTraveled);
            super.robot.getTelemetry().addData("Target Counts", targetCounts);


            lastEncoderValue = encoderValue;
            stopwatch.restart();
        }
    }

}
