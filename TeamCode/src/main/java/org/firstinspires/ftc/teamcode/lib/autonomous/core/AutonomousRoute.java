package org.firstinspires.ftc.teamcode.lib.autonomous.core;

import org.firstinspires.ftc.teamcode.lib.robot.Bot;

import java.util.ArrayList;
import java.util.List;

public class AutonomousRoute {

    private AutonomousMeccanumDrive drive;
    private List<AutonomousInstruction> instructions;

    public AutonomousRoute(Bot robot) {
        this.drive = new AutonomousMeccanumDrive(robot);
        this.instructions = new ArrayList<AutonomousInstruction>();
    }

    public void addInstruction(AutonomousInstruction instruction) {
        this.instructions.add(instruction);
        instruction.setDrive(drive);
    }

    public void execute() {
        for(AutonomousInstruction instruction : instructions) {
            drive.setInstruction(instruction);
            instruction.execute();
        }
    }

}
