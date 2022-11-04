package org.firstinspires.ftc.teamcode.lib.robot;

public class MeccanumDrive {

    private float maxPowerPerSecond;

    private class PowerController {

        private float targetPower;
        private float lastPower;

        private long lastTimeMillis;

        public float getPower() {
            if(lastTimeMillis == 0) {
                lastTimeMillis = System.currentTimeMillis();
                return 0;
            }

            float seconds = (System.currentTimeMillis() - lastTimeMillis) / 1000.0f;
            float powerPerSecond = (targetPower - lastPower) / seconds;

            if(Math.abs(powerPerSecond) > maxPowerPerSecond) {
                int direction = (int) (Math.abs(powerPerSecond) / powerPerSecond);
                lastPower += direction * maxPowerPerSecond * seconds;
            } else {
                lastPower = targetPower;
            }

            lastTimeMillis = System.currentTimeMillis();
            return lastPower;
        }

        public void setPower(float targetPower) {
            this.targetPower = targetPower;
        }

    }

    protected Bot robot;

    private PowerController frontLeftPower;
    private PowerController backLeftPower;
    private PowerController frontRightPower;
    private PowerController backRightPower;

    public MeccanumDrive(Bot robot, float maxPowerPerSecond) {
        this.robot = robot;
        this.maxPowerPerSecond = maxPowerPerSecond;
        this.frontLeftPower = new PowerController();
        this.backLeftPower = new PowerController();
        this.frontRightPower = new PowerController();
        this.backRightPower = new PowerController();
    }

    public Bot getRobot() {
        return this.robot;
    }

    public void applyPower() {
        robot.setFrontLeftDrivePower(frontLeftPower.getPower());
        robot.setBackLeftDrivePower(backLeftPower.getPower());
        robot.setFrontRightDrivePower(frontRightPower.getPower());
        robot.setBackRightDrivePower(backRightPower.getPower());
    }

    public void setDrive(float drive, float turn, float strafe) {
        double totalControls = Math.abs(turn) + Math.abs(drive) + Math.abs(strafe);

        if(totalControls > 1) {
            turn /= totalControls;
            drive /= totalControls;
            strafe /= totalControls;
        }

        float leftPower = drive + turn;
        float rightPower = drive - turn;

        frontLeftPower.setPower(leftPower + strafe);
        backLeftPower.setPower(leftPower - strafe);

        frontRightPower.setPower(rightPower - strafe);
        backRightPower.setPower(rightPower + strafe);
    }

}
