package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.Light;
import org.firstinspires.ftc.teamcode.parts.Orientation;
import org.firstinspires.ftc.teamcode.parts.PIDFPanning;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.PanningServo;
import org.firstinspires.ftc.teamcode.parts.Pitching;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous
public class NoPreSubmersible extends LinearOpMode {
    public PIDFPanning panning;
    public PIDFSlide slides;
    public Claw claw;
    public Orientation orientation;
    public Pitching pitching;
    public PanningServo panningServo;
    public Light light;

    private Timer pathTimer, opmodeTimer, waitTimer;
    int score = 1700;
    int pick = 0;

    double panningScore = 0.9;
    double panningPick = 0.55;

    int slideScore = 915;
    int slideStart = 1025;
    int slideWeird = 1010;

    private Follower follower;
    private final Pose startPose = new Pose(134.937, 78.948, Math.toRadians(0));

    private PathChain goToTwo, pushTwo, pushLast, longHang,
                      hang, back, entireThing;

    private int pathState;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 333:
                follower.followPath(entireThing);
                setPathState(23423);
                break;
            case 0:
                follower.followPath(goToTwo);
                claw.openClaw();
                setPathState(1);
                break;
            case 1:
                if (follower.atParametricEnd()) {
                    follower.followPath(pushTwo);
                    slides.setTargetPos(0);
                    panning.setTargetPos(pick);
                    setPathState(2);
                }
                break;
            case 2:
                if (follower.atParametricEnd()) {
                    follower.followPath(pushLast);
                    claw.closeClaw();
                    panningServo.moveSpecific(panningPick);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 550) {
                    claw.openClaw();
                    setPathState(5);
                }
                break;
            case 5:
                if (follower.getCurrentTValue() > 0.8) {
                    follower.breakFollowing();
                    claw.closeClaw();
                    waitTimer(100);
                    panningServo.moveSpecific(panningScore);
                    orientation.moveOpposite();
                    follower.followPath(longHang);
                    panning.setTargetPos(score);
                    while (panning.getCurrentPos() < 1300) {
                        updateImportant();
                    }
                    slides.setTargetPos(slideWeird);
                    setPathState(6);
                }
                break;
            case 6:
                if (follower.getCurrentTValue() > 0.85) {
                    slides.setTargetPos(0);
                    while (slides.getCurrentPos() > 350) {
                        updateImportant();
                    }
                    follower.followPath(back);
                    claw.openClaw();
                    orientation.moveNormal();
                    panning.setTargetPos(0);
                    panningServo.moveSpecific(panningPick);
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                if (follower.getCurrentTValue() > 0.95) {
                    follower.breakFollowing();
                    claw.closeClaw();
                    waitTimer(250);
                    follower.followPath(hang);
                    orientation.moveOpposite();
                    panningServo.moveSpecific(0.9);
                    panning.setTargetPos(1800);
                    while (panning.getCurrentPos() < 1300) {
                        updateImportant();
                    }
                    slides.setTargetPos(slideScore);
                    setPathState(6);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        goToTwo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(134.937, 78.948, Point.CARTESIAN),
                                new Point(119.631, 115.804, Point.CARTESIAN),
                                new Point(82.573, 102.310, Point.CARTESIAN),
                                new Point(85.594, 118.221, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pushTwo = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(108, 78.948, Point.CARTESIAN),
                                new Point(119.631, 115.804, Point.CARTESIAN),
                                new Point(82.573, 102.310, Point.CARTESIAN),
                                new Point(85.594, 118.221, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(85.594, 118.221, Point.CARTESIAN),
                                new Point(115.603, 118.221, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(115.603, 118.221, Point.CARTESIAN),
                                new Point(82.976, 119.027, Point.CARTESIAN),
                                new Point(84.990, 128.291, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(84.990, 128.291, Point.CARTESIAN),
                                new Point(115.603, 127.888, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(115.603, 127.888, Point.CARTESIAN),
                                new Point(84.587, 127.888, Point.CARTESIAN),
                                new Point(87.206, 135.743, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        pushLast = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(87.206, 135.743, Point.CARTESIAN),
                                new Point(132.319, 135.944, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(4).build();
        longHang = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(132.319, 135.944, Point.CARTESIAN),
                                new Point(127.485, 71.295, Point.CARTESIAN),
                                new Point(112.380, 87.004, Point.CARTESIAN),
                                new Point(112, 79.150, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        hang = follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(133.527, 116.006, Point.CARTESIAN),
                                new Point(124.800, 80.718, Point.CARTESIAN),
                                new Point(111.282, 81.110, Point.CARTESIAN),
                                new Point(112, 76.800, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        back = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(112, 79.150, Point.CARTESIAN),
                                new Point(121.861, 80.131, Point.CARTESIAN),
                                new Point(108.147, 103.445, Point.CARTESIAN),
                                new Point(133.527, 116.006, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(1)
                .build();


        entireThing = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(134.937, 78.948, Point.CARTESIAN),
                                new Point(106.338, 79.150, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(106.338, 79.150, Point.CARTESIAN),
                                new Point(119.631, 115.804, Point.CARTESIAN),
                                new Point(82.573, 102.310, Point.CARTESIAN),
                                new Point(85.594, 118.221, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(85.594, 118.221, Point.CARTESIAN),
                                new Point(115.603, 118.221, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(115.603, 118.221, Point.CARTESIAN),
                                new Point(82.976, 119.027, Point.CARTESIAN),
                                new Point(84.990, 128.291, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(84.990, 128.291, Point.CARTESIAN),
                                new Point(115.603, 127.888, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(115.603, 127.888, Point.CARTESIAN),
                                new Point(84.587, 127.888, Point.CARTESIAN),
                                new Point(87.004, 134.736, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(87.004, 134.736, Point.CARTESIAN),
                                new Point(131.110, 134.937, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(131.110, 134.937, Point.CARTESIAN),
                                new Point(129.499, 75.726, Point.CARTESIAN),
                                new Point(106.137, 76.531, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(106.137, 76.531, Point.CARTESIAN),
                                new Point(121.645, 88.213, Point.CARTESIAN),
                                new Point(131.513, 108.755, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(131.513, 108.755, Point.CARTESIAN),
                                new Point(123.055, 69.885, Point.CARTESIAN),
                                new Point(106.137, 73.712, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(106.137, 73.712, Point.CARTESIAN),
                                new Point(122.048, 88.817, Point.CARTESIAN),
                                new Point(131.513, 108.957, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
    }

    @Override
    public void runOpMode() {
        // Get things from hardware map
        slides = new PIDFSlide(hardwareMap);
        panning = new PIDFPanning(hardwareMap);
        claw = new Claw(hardwareMap);
        pitching = new Pitching(hardwareMap);
        orientation = new Orientation(hardwareMap);
        panningServo = new PanningServo(hardwareMap);
        light = new Light(hardwareMap);

        // Set up follower
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setMaxPower(0.95);
        buildPaths();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        waitTimer = new Timer();

        // Place parts in initial positions
        claw.closeClaw(); // Closed claw
        pitching.moveUp(); // Pitching UP
        orientation.moveNormal(); // Orientation at normal pos

        panning.setTargetPos(0);
        slides.setTargetPos(0);
        light.goToBlue();

        // Set path state to initial
        setPathState(0);

        while (opModeInInit()) {
            slides.updateSlide();
            panning.updatePanning();
            slides.updatePower();
            telemetry.addData("current position: ", panning.getCurrentPos());
            telemetry.update();
        }

        waitForStart();
        follower.setPose(startPose);
        opmodeTimer.resetTimer();
        waitTimer.resetTimer();

        while (opModeIsActive()) {
            autonomousPathUpdate();
            follower.update();
            panning.updatePanning();
            slides.updateSlide();
            slides.updatePower();

            // Feedback to Driver Hub
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("T value: ", follower.getCurrentTValue());
            telemetry.addData("Panning value: ", panning.getCurrentPos());
            telemetry.addData("Slide target: ", slides.getTargetPos());
            telemetry.addData("Slide pos: ", slides.getCurrentPos());
            telemetry.update();
        }
    }

    public void updateImportant() {
        panning.updatePanning();
        slides.updateSlide();
        slides.updatePower();
        follower.update();
    }

    public void waitTimer(int timeMs) {
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() < timeMs) {}
    }
}
