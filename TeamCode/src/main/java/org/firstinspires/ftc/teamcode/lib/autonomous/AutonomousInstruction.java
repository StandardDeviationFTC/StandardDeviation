package org.firstinspires.ftc.teamcode.lib.autonomous;

import org.firstinspires.ftc.teamcode.lib.robot.Bot;

public abstract class AutonomousInstruction {

    protected AutonomousMeccanumDrive drive;
    protected Bot robot;

    public void setDrive(AutonomousMeccanumDrive drive) {
        this.drive = drive;
        this.robot = drive.getRobot();
    }

    public abstract void execute();

}
