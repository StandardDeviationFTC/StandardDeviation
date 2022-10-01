package org.firstinspires.ftc.teamcode.lib.pid;

public class PIDLinearInput implements PIDInput {

    public double value;

    public PIDLinearInput(double value) {
        this.value = value;
    }

    @Override
    public double getErrorFrom(PIDInput other) {
        PIDLinearInput linearInput = (PIDLinearInput) other;
        return this.value - linearInput.value;
    }
}
