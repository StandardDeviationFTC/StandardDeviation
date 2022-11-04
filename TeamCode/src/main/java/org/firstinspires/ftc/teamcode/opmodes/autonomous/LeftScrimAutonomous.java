package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.autonomous.core.AutonomousMeccanumDrive;
import org.firstinspires.ftc.teamcode.lib.autonomous.core.AutonomousRoute;
import org.firstinspires.ftc.teamcode.lib.autonomous.instructions.DriveDistanceInstruction;
import org.firstinspires.ftc.teamcode.lib.autonomous.core.DriveMode;
import org.firstinspires.ftc.teamcode.lib.autonomous.core.ReferenceFrame;
import org.firstinspires.ftc.teamcode.lib.misc.Constants;
import org.firstinspires.ftc.teamcode.lib.robot.Bot;
import org.firstinspires.ftc.teamcode.lib.robot.MeccanumDrive;

@Autonomous(name="Left Scrim Autonomous", group="Linear Opmode")
public class LeftScrimAutonomous extends LinearOpMode {

    private Bot robot;
    private MeccanumDrive drive;
    private ElapsedTime waitTimer;

    @Override
    public void runOpMode() {
        this.robot = new Bot(this);
        this.drive = new MeccanumDrive(robot, 10000000);
        waitTimer = new ElapsedTime();

        telemetry.addData("Status", "Calibrating");
        telemetry.update();

        while(!robot.isCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.closeClaw();
        robot.setFlipperPosition(0);

        waitForStart();
        /*
        long timer = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - timer < 500) {
            robot.setLiftPower(0.5f);
        }
        robot.setLiftPower(0);

        robot.resetEncoders();
        while(opModeIsActive() && getAvgEncoderValue() < 20) {
            drive.setDrive(0.1f, 0, 0);
            drive.applyPower();
        }

        drive.setDrive(0, 0, 0);
        drive.applyPower();

        timer = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - timer < 500);

        robot.resetEncoders();
        while(opModeIsActive() && getAvgStrafeEncoderValue() < 1600) {
            drive.setDrive(0, 0, 0.5f);
            drive.applyPower();
        }

        drive.setDrive(0, 0, 0);
        drive.applyPower();

        timer = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - timer < 500);

        robot.resetEncoders();
        while(opModeIsActive() && getAvgEncoderValue() < 1050) {
            drive.setDrive(0.7f, 0, 0);
            drive.applyPower();
        }

        drive.setDrive(0, 0, 0);
        drive.applyPower();

        timer = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - timer < 500);

        timer = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - timer < 3500) {
            robot.setLiftPower(0.5f);
        }
        robot.setLiftPower(0);

        timer = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - timer < 500);

        robot.resetEncoders();
        while(opModeIsActive() && getAvgEncoderValue() < 260) {
            drive.setDrive(0.1f, 0, 0);
            drive.applyPower();
        }

        timer = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - timer < 500) {
            robot.setLiftPower(-0.5f);
        }
        robot.setLiftPower(0);

        drive.setDrive(0, 0, 0);
        drive.applyPower();

        timer = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - timer < 500);

        robot.openClaw();

        timer = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - timer < 1000);

        robot.resetEncoders();
        while(opModeIsActive() && getAvgEncoderValue() > -100) {
            drive.setDrive(-0.5f, 0, 0);
            drive.applyPower();
        }

        robot.closeClaw();
        drive.setDrive(0, 0, 0);
        drive.applyPower();

        timer = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - timer < 2500) {
            robot.setLiftPower(-0.5f);
        }
        robot.setLiftPower(0);

        robot.resetEncoders();
        while(opModeIsActive() && getAvgEncoderValue() > -1100) {
            drive.setDrive(-0.7f, 0, 0);
            drive.applyPower();
        }

        drive.setDrive(0, 0, 0);
        drive.applyPower();
        */

        //setup
        runLift(-0.5f, 500);
        driveDistance(0.1f, 20);
        wait(500);

        //drive close to pole
        strafeDistance(0.5f, 1600);
        wait(500);
        driveDistance(0.7f, 1050);
        wait(500);

        //raise lift and inch closer
        runLift(-0.5f, 3500);
        wait(500);
        driveDistance(0.1f, 260);

        //lower cone onto pole
        runLift(0.5f, 500);
        wait(500);

        //drop cone
        robot.openClaw();
        wait(1000);

        //back away from pole
        runLift(-0.5f, 500);
        driveDistance(-0.5f, 200);

        //close claw and lower lift
        robot.closeClaw();
        runLift(0.5f, 2500);

        //park
        strafeDistance(-0.5f, 525);

        while(opModeIsActive());
    }

    private int sign(float val) {
        return (int) (Math.abs(val) / val);
    }

    private void wait(int millis) {
        waitTimer.reset();
        while(opModeIsActive() && waitTimer.milliseconds() < millis);
    }

    private void runLift(float power, int millis) {
        robot.setLiftPower(power);
        waitTimer.reset();
        while(opModeIsActive() && waitTimer.milliseconds() < millis) {
            if(robot.getLiftLimitSwitchState()) robot.setLiftPower(Math.max(0, power));
        }
        robot.setLiftPower(0);
    }

    private void strafeDistance(float power, int tgtEncoderTicks) {
        int direction = sign(power);

        robot.resetEncoders();
        while(opModeIsActive() && direction * getAvgStrafeEncoderValue() - tgtEncoderTicks < 0) {
            drive.setDrive(0, 0, power);
            drive.applyPower();
        }

        drive.setDrive(0, 0, 0);
        drive.applyPower();
    }

    private void driveDistance(float power, int tgtEncoderTicks) {
        int direction = sign(power);

        robot.resetEncoders();
        while(opModeIsActive() && direction * getAvgEncoderValue() - tgtEncoderTicks < 0) {
            drive.setDrive(power, 0, 0);
            drive.applyPower();
        }

        drive.setDrive(0, 0, 0);
        drive.applyPower();
    }

    private float getAvgStrafeEncoderValue() {
        return 0.25f * (-robot.getBackLeftEncoder() + robot.getBackRightEncoder() + robot.getFrontLeftEncoder() - robot.getFrontRightEncoder());
    }

    private float getAvgEncoderValue() {
        return 0.25f * (robot.getBackLeftEncoder() + robot.getBackRightEncoder() + robot.getFrontLeftEncoder() + robot.getFrontRightEncoder());
    }
}
