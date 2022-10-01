package org.firstinspires.ftc.teamcode.lib.pid;

public class PIDController {

    private double p;
    private double i;
    private double d;

    private double integral;

    private long lastTimeMillis;
    private double lastError;

    private PIDInput target;
    private PIDInput input;
    private boolean hasUpdated;

    public PIDController(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;

        lastTimeMillis = System.currentTimeMillis();
    }

    public double getCurrentError() {
        return input.getErrorFrom(target);
    }

    public void reset() {
        integral = 0;
        hasUpdated = false;
        lastTimeMillis = System.currentTimeMillis();
    }

    public void setInput(PIDInput input) {
        this.input = input;
    }

    public void setTarget(PIDInput target) {
        this.target = target;
    }

    public double getResponse() {
        double dt = (System.currentTimeMillis() - lastTimeMillis) / 1000.0;
        double error = input.getErrorFrom(target);
        double derivative = (error - lastError) / dt;

        integral += error * dt;

        lastTimeMillis = System.currentTimeMillis();
        lastError = error;

        if(hasUpdated)
            return p * (error + i * integral + d * derivative);

        hasUpdated = true;
        return 0;
    }

}
