package org.firstinspires.ftc.teamcode.lib.autonomous;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.lib.robot.Bot;
import org.firstinspires.ftc.teamcode.lib.misc.Constants;

import java.util.ArrayList;
import java.util.List;

public class VuforiaNavigation {

    private static final String VUFORIA_KEY =
            "AUwWd/X/////AAABmXBvwC4YtUYhobPZmBANtUVmen6Q/60keakx5OUvCFzszapZQRBAzVDNzgxCOT7gCavUPLDTEInvUIhr07bDBq082eth97lNKwMGFGQtrnxCnL8bueTXgQ/C9/Hz3kQ4aMu42ZYskWCQSIlArOOPYbQVfujc+5415hGR7H9JB7qNTsZw4XLr/wSUbzrdQimGr0SvsIFTEuKrBpjdguhSSfi1hiAbYnY8YEyV76JHqtYq1S+VXL//Za0MbdBCJI8+ZSWgreEdCqr1nqnzHpUmxzQADhQ87n4DpZNgM00GXHPel/041ft+cyMcRZz2Ubny4rjAw96coVc6M95wfenHaqHnHchaZtmg4gYXggqxpKwd";

    private static final String WEBCAM_CONFIG_NAME = "Webcam 1";

    private static final float MEASUREMENT_FRAME_TIME_MILLIS = 100;

    private OpenGLMatrix relativeCameraLocation;

    private OpenGLMatrix lastLocation;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables targets;

    private List<VuforiaTrackable> trackables;
    private VuforiaTrackable trackedTarget;

    private long measurementFrameStartMillis;

    public VuforiaNavigation(Bot robot) {
        HardwareMap hardwareMap = robot.getHardwareMap();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, WEBCAM_CONFIG_NAME);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = createVuforia(webcamName, cameraMonitorViewId);
        createTargets();
        setCameraLocation(parameters);
    }

    private VuforiaLocalizer.Parameters createVuforia(WebcamName webcamName, int cameraMonitorViewID) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewID);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        return parameters;
    }

    private void setCameraLocation(VuforiaLocalizer.Parameters parameters) {
        relativeCameraLocation = OpenGLMatrix
                .translation(Constants.CAMERA_FORWARD_DISPLACEMENT, Constants.CAMERA_LEFT_DISPLACEMENT, Constants.CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        for (VuforiaTrackable trackable : trackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, relativeCameraLocation);
        }

        targets.activate();
    }

    private void createTargets() {
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        trackables = new ArrayList<VuforiaTrackable>();
        trackables.addAll(targets);

        identifyTarget(0, "Blue Alliance Audience",
                -Constants.HALF_FIELD_MM,  Constants.ONE_AND_HALF_TILE_MM, Constants.TARGET_HEIGHT_MM, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Rear",
                Constants.HALF_TILE_MM, Constants.HALF_FIELD_MM, Constants.TARGET_HEIGHT_MM, 90, 0, 0);
        identifyTarget(2, "Red Alliance Audience",
                -Constants.HALF_FIELD_MM, -Constants.ONE_AND_HALF_TILE_MM, Constants.TARGET_HEIGHT_MM, 90, 0, 90);
        identifyTarget(3, "Red Alliance Rear",
                Constants.HALF_TILE_MM, -Constants.HALF_FIELD_MM, Constants.TARGET_HEIGHT_MM, 90, 0, 180);
    }

    public VuforiaTrackable getTrackedTarget() {
        return this.trackedTarget;
    }

    public void cleanUp() {
        targets.deactivate();
    }

    public OpenGLMatrix getLastLocation() {
        return this.lastLocation;
    }

    public void update() {
        trackedTarget = null;

        for (VuforiaTrackable trackable : trackables) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            if (listener.isVisible()) {
                trackedTarget = trackable;

                OpenGLMatrix robotLocation = listener.getUpdatedRobotLocation();
                if (robotLocation != null) {
                    lastLocation = robotLocation;
                }
                break;
            }
        }
    }


    private void identifyTarget(int targetIndex, String targetName, float x, float y, float z, float rx, float ry, float rz) {
        VuforiaTrackable target = targets.get(targetIndex);
        target.setName(targetName);
        target.setLocation(OpenGLMatrix.translation(x, y, z)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

}
