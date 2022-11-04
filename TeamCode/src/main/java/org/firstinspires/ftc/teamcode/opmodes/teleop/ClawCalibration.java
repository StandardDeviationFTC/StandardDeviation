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
import org.firstinspires.ftc.teamcode.lib.input.Input;
import org.firstinspires.ftc.teamcode.lib.input.ToggleButton;
import org.firstinspires.ftc.teamcode.lib.misc.Constants;
import org.firstinspires.ftc.teamcode.lib.input.Controls;
import org.firstinspires.ftc.teamcode.lib.misc.Util;
import org.firstinspires.ftc.teamcode.lib.pid.PIDAngleInput;
import org.firstinspires.ftc.teamcode.lib.pid.PIDController;
import org.firstinspires.ftc.teamcode.lib.robot.Bot;
import org.firstinspires.ftc.teamcode.lib.robot.MeccanumDrive;

@TeleOp(name="Claw Calibration", group="Linear Opmode")
@Disabled
public class ClawCalibration extends LinearOpMode {

    private Bot robot;

    private static final float CALIBRATION_STEP = 0.01f;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot = new Bot(this);

        Controls.setGamepad(gamepad1);
        Controls.bindInput("LeftClose", Input.DPAD_RIGHT);
        Controls.bindInput("LeftOpen", Input.DPAD_LEFT);
        Controls.bindInput("RightClose", Input.DPAD_UP);
        Controls.bindInput("RightOpen", Input.DPAD_DOWN);
        Controls.bindInput("FlipperLeft", Input.LB);
        Controls.bindInput("FlipperRight", Input.RB);

        ToggleButton leftClose = new ToggleButton("LeftClose");
        ToggleButton leftOpen = new ToggleButton("LeftOpen");
        ToggleButton rightClose = new ToggleButton("RightClose");
        ToggleButton rightOpen = new ToggleButton("RightOpen");
        ToggleButton flipperLeft = new ToggleButton("FlipperLeft");
        ToggleButton flipperRight = new ToggleButton("FlipperRight");

        float leftPosition = Constants.LEFT_CLAW_CLOSED_POSITION;
        float rightPosition = Constants.RIGHT_CLAW_CLOSED_POSITION;
        float flipperPosition = Constants.FLIPPER_START_POSITION;

        while(!robot.isCalibrated()) {
            sleep(50);
            idle();
        }

        waitForStart();

        while (opModeIsActive()) {
            if(leftClose.update()) {
                leftPosition -= CALIBRATION_STEP;
            }
            if(leftOpen.update()) {
                leftPosition += CALIBRATION_STEP;
            }
            if(rightClose.update()) {
                rightPosition += CALIBRATION_STEP;
            }
            if(rightOpen.update()) {
                rightPosition -= CALIBRATION_STEP;
            }
            if(flipperLeft.update()) {
                flipperPosition -= CALIBRATION_STEP;
            }
            if(flipperRight.update()) {
                flipperPosition += CALIBRATION_STEP;
            }

            robot.setFlipperPosition(flipperPosition);
            robot.setClawPositions(leftPosition, rightPosition);
            telemetry.addData("Flipper Position", flipperPosition);
            telemetry.addData("Left Claw Position", leftPosition);
            telemetry.addData("Right Claw Position", rightPosition);

            robot.update();
        }

        robot.cleanUp();
    }
}
