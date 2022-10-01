package org.firstinspires.ftc.teamcode.lib.autonomous;

import java.util.ArrayList;
import java.util.List;

public class AutonomousRoute {

    private AutonomousMeccanumDrive drive;
    private List<AutonomousInstruction> instructions;

    public AutonomousRoute(AutonomousMeccanumDrive drive) {
        this.drive = drive;
        this.instructions = new ArrayList<AutonomousInstruction>();
    }

    public void addInstruction(AutonomousInstruction instruction) {
        this.instructions.add(instruction);
        instruction.setDrive(drive);
    }

    public void execute() {
        for(AutonomousInstruction instruction : instructions)
            instruction.execute();
    }

}
