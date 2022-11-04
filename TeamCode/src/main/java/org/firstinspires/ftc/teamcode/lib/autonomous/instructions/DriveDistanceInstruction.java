package org.firstinspires.ftc.teamcode.lib.autonomous.instructions;

import org.firstinspires.ftc.teamcode.lib.autonomous.core.AutonomousInstruction;
import org.firstinspires.ftc.teamcode.lib.autonomous.core.DriveMode;
import org.firstinspires.ftc.teamcode.lib.misc.Constants;

public class DriveDistanceInstruction extends AutonomousInstruction {

    private float inches;

    private DriveMode mode;

    public DriveDistanceInstruction(DriveMode mode, float inches) {
        this.inches = inches;
        this.mode = mode;
    }

    @Override
    public void execute() {

        float lastPosition = mode.isDrive() ? super.drive.getFieldY() : super.drive.getFieldX();
        long lastTimeMillis = System.currentTimeMillis();

        super.drive.driveDistance(mode, inches);
        while(super.robot.opModeIsActive()) {
            super.drive.update();

            long currentTimeMillis = System.currentTimeMillis();
            float elapsedSeconds = (currentTimeMillis - lastTimeMillis) / 1000.0f;

            float currentPosition = mode.isDrive() ? super.drive.getFieldY() : super.drive.getFieldX();
            float speed = Math.abs(currentPosition - lastPosition) / elapsedSeconds;

            if(super.drive.getPositionError() < Constants.AUTO_DRIVE_DISTANCE_TOLERANCE_INCHES &&
                    speed < Constants.AUTO_DRIVE_DISTANCE_TOLERANCE_INCHES_PER_SECOND) {
                break;
            }

            lastPosition = mode.isDrive() ? super.drive.getFieldY() : super.drive.getFieldX();
            lastTimeMillis = System.currentTimeMillis();
        }
    }

}
