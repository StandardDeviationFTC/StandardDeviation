package org.firstinspires.ftc.teamcode.lib.autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.lib.misc.Constants;

public class AutonomousHeadingInstruction extends AutonomousInstruction {

    private float targetHeading;

    public AutonomousHeadingInstruction(float heading) {
        this.targetHeading = heading;
    }

    @Override
    public void execute() {
        super.drive.setTargetHeading(targetHeading);

        while(super.robot.opModeIsActive()) {
            super.drive.update();

            float heading = super.robot.getOrientationDeg(AxesOrder.XYZ).thirdAngle;
            if(Math.abs(heading - targetHeading) < Constants.AUTO_TURN_HEADING_TOLERANCE_DEG)
                break;
        }
    }
}
