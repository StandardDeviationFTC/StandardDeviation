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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.lib.autonomous.VuforiaNavigation;
import org.firstinspires.ftc.teamcode.lib.input.Input;
import org.firstinspires.ftc.teamcode.lib.misc.Constants;
import org.firstinspires.ftc.teamcode.lib.input.Controls;
import org.firstinspires.ftc.teamcode.lib.misc.Util;
import org.firstinspires.ftc.teamcode.lib.pid.PIDAngleInput;
import org.firstinspires.ftc.teamcode.lib.pid.PIDController;
import org.firstinspires.ftc.teamcode.lib.robot.Bot;
import org.firstinspires.ftc.teamcode.lib.robot.MeccanumDrive;

@TeleOp(name="Rook", group="Linear Opmode")
public class RookOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Bot robot;
    private MeccanumDrive meccanumDrive;
    private PIDController pid;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        pid = new PIDController(-0.016, 0.5, 0.2);
        robot = new Bot(this);
        meccanumDrive = new MeccanumDrive(robot);

        Controls.setGamepad(gamepad1);
        Controls.bindInput("MoveY", Input.LEFT_STICK_X);
        Controls.bindInput("MoveX", Input.LEFT_STICK_Y);
        Controls.bindInput("Turn", Input.RIGHT_STICK_X);

        while(!robot.isCalibrated()) {
            sleep(50);
            idle();
        }

        waitForStart();
        runtime.reset();

        long lastTime = System.currentTimeMillis();
        double zOrientation = robot.getOrientationDeg(AxesOrder.XYZ).thirdAngle;

        PIDAngleInput targetAngle = new PIDAngleInput(zOrientation);
        PIDAngleInput currentAngle = new PIDAngleInput(zOrientation);
        pid.setTarget(targetAngle);
        pid.setInput(currentAngle);

        while (opModeIsActive()) {
            robot.update();

            double deltaTime = (System.currentTimeMillis() - lastTime) / 1000.0;
            lastTime = System.currentTimeMillis();

            targetAngle.setAngle(targetAngle.getAngle() + -200 * applyCurve(Controls.getFloat("Turn")) * deltaTime);

            float stickX = applyCurve(-Controls.getFloat("MoveX"));
            float stickY = applyCurve(Controls.getFloat("MoveY"));

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

            VuforiaNavigation vuforiaNavigation = robot.getVuforiaNavigation();
            VuforiaTrackable trackedTarget = vuforiaNavigation.getTrackedTarget();
            boolean targetVisible = false;
            if(trackedTarget != null) {
                targetVisible = true;
                telemetry.addData("Visible Target: ", trackedTarget.getName());
                OpenGLMatrix location = vuforiaNavigation.getLastLocation();
                VectorF translation = location.getTranslation();

                telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / Constants.MM_PER_INCH, translation.get(1) / Constants.MM_PER_INCH, translation.get(2) / Constants.MM_PER_INCH);

                Orientation rotation = Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }

            telemetry.addData("Target Visible:", targetVisible);

            telemetry.update();
        }

        robot.cleanUp();
    }

    private float getTurn(PIDAngleInput currentAngle, PIDAngleInput targetAngle) {
        currentAngle.setAngle(robot.getOrientationDeg(AxesOrder.XYZ).thirdAngle);

        return (float) pid.getResponse();
    }

    private float applyCurve(float d) {
        int sign = (int) (Math.abs(d)/d);
        return sign * d * d;
    }
}
