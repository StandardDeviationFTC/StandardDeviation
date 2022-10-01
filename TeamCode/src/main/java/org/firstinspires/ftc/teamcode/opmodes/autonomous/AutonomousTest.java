package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.autonomous.AutonomousDriveInstruction;
import org.firstinspires.ftc.teamcode.lib.autonomous.AutonomousHeadingInstruction;
import org.firstinspires.ftc.teamcode.lib.autonomous.AutonomousMeccanumDrive;
import org.firstinspires.ftc.teamcode.lib.autonomous.AutonomousRoute;
import org.firstinspires.ftc.teamcode.lib.robot.Bot;

@Autonomous(name="Autonomous", group="Linear Opmode")
public class AutonomousTest extends LinearOpMode {

    private Bot robot;
    private AutonomousRoute route;
    private AutonomousMeccanumDrive drive;

    @Override
    public void runOpMode() {
        this.robot = new Bot(this);
        this.drive = new AutonomousMeccanumDrive(robot);
        this.route = new AutonomousRoute(drive);

        route.addInstruction(new AutonomousDriveInstruction(30));
        //route.addInstruction(new AutonomousHeadingInstruction(90));
        //route.addInstruction(new AutonomousDriveInstruction(20));

        telemetry.addData("Status", "Calibrating");
        telemetry.update();

        while(!robot.isCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        route.execute();
        robot.cleanUp();

    }
}
