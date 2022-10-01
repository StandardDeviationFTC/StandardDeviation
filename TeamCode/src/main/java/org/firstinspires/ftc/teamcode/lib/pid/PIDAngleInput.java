package org.firstinspires.ftc.teamcode.lib.pid;

public class PIDAngleInput implements PIDInput {

    private double angle;

    public PIDAngleInput(double angle) {
        this.angle = angle;
    }

    public void setAngle(double newAngle) {
        while(newAngle < -180) newAngle += 360;
        while(newAngle > 180) newAngle -= 360;
        this.angle = newAngle;
    }

    public double getAngle() {
        return this.angle;
    }

    @Override
    public double getErrorFrom(PIDInput other) {
        PIDAngleInput angleInput = (PIDAngleInput) other;
        double diff = angleInput.angle - this.angle;
        if(diff > 180) diff = -360 + diff;
        else if(diff < -180) diff = 360 + diff;

        return diff;
    }

}
