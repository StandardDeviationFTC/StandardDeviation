package org.firstinspires.ftc.teamcode.lib.autonomous.core;

public enum DriveMode {
    LEFT(false, -1), RIGHT(false, 1),
    FORWARDS(true, 1), BACKWARDS(false, -1);

    private boolean isDrive;
    private int direction;

    private DriveMode(boolean isDrive, int direction) {
        this.isDrive = isDrive;
        this.direction = direction;
    }

    public int getDirection() {
        return this.direction;
    }

    public boolean isDrive() {
        return this.isDrive;
    }

    public boolean isStrafe() {
        return !this.isDrive;
    }
}
