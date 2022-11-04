package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.robot.Bot;
import org.firstinspires.ftc.teamcode.lib.robot.MeccanumDrive;

@Autonomous(name="Right Scrim Autonomous", group="Linear Opmode")
public class RightScrimAutonomous extends LinearOpMode {

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

        //setup
        runLift(-0.5f, 500);
        driveDistance(0.1f, 20);
        wait(500);

        //drive close to pole
        strafeDistance(-0.5f, 1600);
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
        strafeDistance(0.5f, 525);

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
