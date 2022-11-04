/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.lib.input.Input;
import org.firstinspires.ftc.teamcode.lib.input.ToggleButton;
import org.firstinspires.ftc.teamcode.lib.misc.Constants;
import org.firstinspires.ftc.teamcode.lib.input.Controls;
import org.firstinspires.ftc.teamcode.lib.misc.Util;
import org.firstinspires.ftc.teamcode.lib.pid.PIDAngleInput;
import org.firstinspires.ftc.teamcode.lib.pid.PIDController;
import org.firstinspires.ftc.teamcode.lib.robot.Bot;
import org.firstinspires.ftc.teamcode.lib.robot.MeccanumDrive;

@TeleOp(name="Rook", group="Linear Opmode")
@Disabled
public class PowerPlay extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Bot robot;
    private MeccanumDrive meccanumDrive;
    private PIDController pid;

    private float liftPosition;
    private boolean clawButtonPressed;
    private boolean clawFlipButtonPressed;

    private float flipperPosition = 0;

    private ToggleButton clawToggle;
    private ToggleButton flipperToggle;
    private ToggleButton freeDriveToggle;
    private ToggleButton directionToggle;

    private PIDAngleInput targetAngle;
    private PIDAngleInput currentAngle;

    private boolean driving;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        pid = new PIDController(-0.025, 0.2, 0.2);
        robot = new Bot(this);
        meccanumDrive = new MeccanumDrive(robot, Constants.MAX_POWER_PER_SECOND);

        initControls();

        while(!robot.isCalibrated()) {
            sleep(50);
            idle();
        }

        waitForStart();
        runtime.reset();

        long lastTime = System.currentTimeMillis();
        long lastIncrementTime = System.currentTimeMillis();
        double zOrientation = robot.getHeading();

        targetAngle = new PIDAngleInput(zOrientation);
        currentAngle = new PIDAngleInput(zOrientation);
        pid.setTarget(targetAngle);
        pid.setInput(currentAngle);

        while (opModeIsActive()) {
            freeDriveToggle.update();
            if(freeDriveToggle.getState()) {
                doFreeDrive();
            } else {
                doGridDrive();
            }

            float liftPower = applyCurve(Controls.getFloat("LiftUp")) - applyCurve(Controls.getFloat("LiftDown"));
            robot.setLiftPower(liftPower);

            controlServos();

            telemetry.addData("Lift Position", liftPosition);
            telemetry.addData("X button", Controls.getBoolean("FlipToggle"));
            telemetry.addData("Flipper position", flipperPosition);
            telemetry.addData("Free Drive", freeDriveToggle.getState());
            telemetry.addData("Turn", Controls.getFloat("Turn"));
            telemetry.addData("Claw open", clawToggle.getState());

            telemetry.addData("Orientation XYZ", robot.getOrientation(AxesOrder.XYZ));
            telemetry.addData("Orientation XZY", robot.getOrientation(AxesOrder.XZY));
            telemetry.addData("Orientation YXZ", robot.getOrientation(AxesOrder.YXZ));
            telemetry.addData("Orientation YZX", robot.getOrientation(AxesOrder.YZX));
            telemetry.addData("Orientation ZXY", robot.getOrientation(AxesOrder.ZXY));
            telemetry.addData("Orientation ZYX", robot.getOrientation(AxesOrder.ZYX));


            telemetry.addData("Heading", robot.getHeading());

            telemetry.addData("Flipper state", flipperToggle.getState());

            robot.update();
        }

        robot.cleanUp();
    }

    private void setFreeDriveControls() {
        Controls.bindInput("Turn", Input.LEFT_STICK_X);
        Controls.bindInput("MoveY", Input.LEFT_STICK_Y);
        Controls.bindInput("MoveX", Input.RIGHT_STICK_X);
    }

    private void setGridDriveControls() {
        Controls.bindInput("MoveY", Input.LEFT_STICK_X);
        Controls.bindInput("MoveX", Input.LEFT_STICK_Y);
        Controls.bindInput("Turn", Input.RIGHT_STICK_X);
    }

    private void doFreeDrive() {
        setFreeDriveControls();
        int direction = directionToggle.getState() ? 1 : -1;

        float drive = applyCurve(-direction * Controls.getFloat("MoveY"));
        float strafe = applyCurve(direction * Controls.getFloat("MoveX"));

        setDriving((drive > 0.05 || strafe > 0.05));

        float turn = getTurn(currentAngle, targetAngle);

        meccanumDrive.setDrive(drive, turn, strafe);
        meccanumDrive.applyPower();
    }

    private void setDriving(boolean newDriving) {
        if(newDriving) {
            if(!driving) {
                driving = true;
                targetAngle.setAngle(robot.getHeading());
            }
        } else {
            pid.reset();
            driving = false;
        }
    }

    private void doGridDrive() {
        setGridDriveControls();
        int direction = directionToggle.getState() ? 1 : -1;

        float stickX = applyCurve(-direction * Controls.getFloat("MoveX"));
        float stickY = applyCurve(direction * Controls.getFloat("MoveY"));

        setDriving((stickX > 0.05 || stickY > 0.05));

        float xDrive = 0;
        float yDrive = 0;

        if(Math.abs(stickX) > Math.abs(stickY)) {
            xDrive = stickX;
            yDrive = stickY * 0.1f;
        } else {
            xDrive = stickX * 0.1f;
            yDrive = stickY;
        }

        VectorF driveStrafeVector = Util.getDriveStrafeVector(xDrive, yDrive, (float) currentAngle.getAngle());

        float turn = getTurn(currentAngle, targetAngle);
        float drive = driveStrafeVector.get(0);
        float strafe = driveStrafeVector.get(1);

        meccanumDrive.setDrive(drive, turn, strafe);
        meccanumDrive.applyPower();
    }

    private void controlServos() {
        clawToggle.update();
        flipperToggle.update();

        if(clawToggle.getState())
            robot.openClaw();
        else
            robot.closeClaw();

        if(flipperToggle.getState())
            robot.setFlipperPosition(0);
        else
            robot.setFlipperPosition(Constants.FLIPPER_ACTUATE_DISTANCE);
    }

    private void initControls() {
        Controls.setGamepad(gamepad1);
        Controls.bindInput("LiftUp", Input.LT);
        Controls.bindInput("LiftDown", Input.RT);
        Controls.bindInput("ClawToggle", Input.CIRCLE);
        Controls.bindInput("FlipToggle", Input.X);
        Controls.bindInput("FreeDriveToggle", Input.TRIANGLE);
        Controls.bindInput("DirectionToggle", Input.SQUARE);

        setGridDriveControls();

        freeDriveToggle = new ToggleButton("FreeDriveToggle", false);
        clawToggle = new ToggleButton("ClawToggle", false);
        flipperToggle = new ToggleButton("FlipToggle", false);
        directionToggle = new ToggleButton("DirectionToggle", false);
    }

    private float getTurn(PIDAngleInput currentAngle, PIDAngleInput targetAngle) {
        if(driving) {
            currentAngle.setAngle(robot.getHeading());
            //return (float) pid.getResponse();
            return 0;
        } else {
            return Controls.getFloat("Turn");
        }
    }

    private float applyCurve(float d) {
        int sign = (int) (Math.abs(d)/d);
        return sign * d * d;
    }
}
