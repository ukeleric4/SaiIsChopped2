package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Vision {
    private final Size CAMERA_RESOLUTION = new Size(320, 240);

    private SampleOrientationProcessorYellow processorYellow;
    private SampleOrientationProcessorBlue processorBlue;
    private SampleOrientationProcessorRed processorRed;
    private ArrayList<Point> offsetRectsYellow;
    private ArrayList<org.opencv.core.Point> offsetRectsBlue;
    private ArrayList<org.opencv.core.Point> offsetRectsRed;
    private WebcamName webcam;
    public VisionPortal visionPortal;

    public double xMovement = 0;
    public double yMovement = 0;

    boolean detectedYellow = false;
    boolean detectedBlue = false;
    boolean detectedRed = false;

    private boolean blueEnabled;
    private boolean yellowEnabled;
    private boolean redEnabled;

    private double orientationPosition = 1.0;
    private double angle = 90;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry, boolean yellowEnabled, boolean blueEnabled, boolean redEnabled) {
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        visionPortal = buildVisionPortal(webcam, telemetry, yellowEnabled, blueEnabled, redEnabled);
        this.blueEnabled = blueEnabled;
        this.yellowEnabled = yellowEnabled;
        this.redEnabled = redEnabled;
    }

    public void updateVision() {
        if (yellowEnabled) {
            offsetRectsYellow = processorYellow.getOffsets();
            detectedYellow = !offsetRectsYellow.isEmpty();
        }
        if (blueEnabled) {
            offsetRectsBlue = processorBlue.getOffsets();
            detectedBlue = !offsetRectsBlue.isEmpty();
        }
        if (redEnabled) {
            offsetRectsRed = processorRed.getOffsets();
            detectedRed = !offsetRectsRed.isEmpty();
        }

        if (detectedYellow) {
            orientationPosition = processorYellow.getServoPosition();
            angle = processorYellow.getSampleAngle();
            updateYValueInches(offsetRectsYellow.get(0).x);
            updateXValueInches(offsetRectsYellow.get(0).y);
        } else if (detectedRed) {
            orientationPosition = processorRed.getServoPosition();
            angle = processorRed.getSampleAngle();
            updateYValueInches(offsetRectsRed.get(0).x);
            updateXValueInches(offsetRectsRed.get(0).y);
        } else if (detectedBlue) {
            orientationPosition = processorBlue.getServoPosition();
            angle = processorBlue.getSampleAngle();
            updateYValueInches(offsetRectsBlue.get(0).x);
            updateXValueInches(offsetRectsBlue.get(0).y);
        } else {
            xMovement = 0;
            yMovement = -2;
        }
    }

    public void updateXValueInches(double xValue) {
        // Checks if the block is to the right or left:
        // Multipliers based on data
        if (xValue > 1) {
            xMovement = (xValue * 4.1) - 2.8; // Left of camera (value in inches)
        } else {
            xMovement = (xValue * 4.1) - 2.8; // Right of camera (value in inches)
        }
    }

    public void updateYValueInches(double yValue) {
        yMovement = (yValue * -3) - 5.25;
    }

    public double getxMovement() {
        return xMovement;
    }

    public double getyMovement() {
        return yMovement;
    }

    public double getOrientation() {
        return orientationPosition;
    }

    public double getAngle() {
        return angle;
    }

    public VisionPortal buildVisionPortal(WebcamName cameraName, Telemetry telemetry, boolean yellowEnabled, boolean blueEnabled, boolean redEnabled) {
        processorYellow = new SampleOrientationProcessorYellow(telemetry);
        processorBlue = new SampleOrientationProcessorBlue(telemetry);
        processorRed = new SampleOrientationProcessorRed(telemetry);

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(cameraName)
                .setCameraResolution(CAMERA_RESOLUTION)
                .setAutoStopLiveView(false)
                .addProcessor(processorYellow)// ADD PROCESSORS HERE
                .addProcessor(processorBlue)
                .addProcessor(processorRed)
                .build();

        visionPortal.setProcessorEnabled(processorYellow, yellowEnabled);
        visionPortal.setProcessorEnabled(processorBlue, blueEnabled);
        visionPortal.setProcessorEnabled(processorRed, redEnabled);
        return visionPortal;
    }

}
