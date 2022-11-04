package org.firstinspires.ftc.teamcode.lib.autonomous.instructions;

import org.firstinspires.ftc.teamcode.lib.autonomous.core.AutonomousInstruction;
import org.firstinspires.ftc.teamcode.lib.misc.Constants;

public class HeadingInstruction extends AutonomousInstruction {

    private float targetHeading;

    public HeadingInstruction(float heading) {
        this.targetHeading = heading;
    }

    @Override
    public void execute() {
        super.drive.setTargetHeading(targetHeading);

        float lastHeadingError = super.drive.getHeadingError();
        long lastTimeMillis = System.currentTimeMillis();

        while(super.robot.opModeIsActive()) {
            super.drive.update();

            long currentTimeMillis = System.currentTimeMillis();
            float elapsedTimeSeconds = (currentTimeMillis - lastTimeMillis) / 1000.0f;
            float currentHeadingError = Math.abs(super.drive.getHeadingError());
            float turnSpeed = Math.abs(currentHeadingError - lastHeadingError) / elapsedTimeSeconds;

            float heading = super.robot.getHeading();
            if(currentHeadingError < Constants.AUTO_TURN_HEADING_TOLERANCE_DEG &&
                    turnSpeed < Constants.AUTO_TURN_HEADING_TOLERANCE_DEG_PER_SECOND) {
                break;
            }

            lastHeadingError = currentHeadingError;
            lastTimeMillis = currentTimeMillis;
        }
    }
}
