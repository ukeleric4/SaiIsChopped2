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
import com.qualcomm.robotcore.hardware.DistanceSensor;

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
public class SubOld extends LinearOpMode {
    public PIDFPanning panning;
    public PIDFSlide slides;
    public Claw claw;
    public Orientation orientation;
    public Pitching pitching;
    public PanningServo panningServo;
    public DistanceSensor dSensor;
    public Light light;

    private Timer pathTimer, opmodeTimer;
    int score = 1700;
    int pick = 0;

    double panningScore = 0.9;
    double panningPick = 0.45;

    int slideScore = 950;
    int slideStart = 1100;
    int slidePickUp = 600;

    private Follower follower;
    private final Pose startPose = new Pose(134.937, 78.948, Math.toRadians(0));

    private PathChain hangFirst, pushTwo, pushLast, longHang, longBack,
                      hang, back, entireThing, backFast;

    private int pathState;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 333:
                follower.followPath(entireThing);
                setPathState(23423);
                break;
            case 0:
                panningServo.moveSpecific(panningScore);
                slides.setTargetPos(slideStart);
                panning.setTargetPos(score);
                setPathState(123);
                break;
            case 123:
                if (pathTimer.getElapsedTime() > 500) {
                    follower.followPath(hangFirst);
                    setPathState(1);
                }
                break;
            case 1:
                if (follower.atParametricEnd()) {
                    slides.setTargetPos(0);
                    while (slides.getCurrentPos() > 650) {
                        slides.updateSlide();
                        slides.updatePower();
                        follower.update();
                    }
                    claw.openClaw();
                    follower.followPath(pushTwo);
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
                if (pathTimer.getElapsedTime() > 500) {
                    claw.openClaw();
                    setPathState(5);
                }
                break;
            case 5:
                if (follower.atParametricEnd() || follower.isRobotStuck()) {
                    follower.breakFollowing();
                    claw.closeClaw();
                    sleep(100);
                    panningServo.moveSpecific(panningScore);
                    orientation.moveOpposite();
                    follower.followPath(longHang);
                    panning.setTargetPos(score);
                    slides.setTargetPos(slideScore);
                    setPathState(6);
                }
                break;
            case 6:
                if (follower.atParametricEnd()) {
                    slides.setTargetPos(0);
                    while (slides.getCurrentPos() > 650) {
                        slides.updateSlide();
                        slides.updatePower();
                        follower.update();
                    }
                    claw.openClaw();
                    follower.followPath(longBack);
                    orientation.moveNormal();
                    panning.setTargetPos(pick);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTime() > 500) {
                    claw.closeClaw();
                    panningServo.moveSpecific(panningPick);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTime() > 500) {
                    claw.openClaw();
                    setPathState(9);
                }
                break;
            case 9:
                if (follower.atParametricEnd() || follower.isRobotStuck()) {
                    follower.breakFollowing();
                    claw.closeClaw();
                    sleep(100);
                    panningServo.moveSpecific(panningScore);
                    orientation.moveOpposite();
                    follower.followPath(hang);
                    panning.setTargetPos(score);
                    slides.setTargetPos(slideScore);
                    setPathState(10);
                }
                break;
            case 10:
                if (follower.atParametricEnd()) {
                    slides.setTargetPos(0);
                    while (slides.getCurrentPos() > 650) {
                        slides.updateSlide();
                        slides.updatePower();
                        follower.update();
                    }
                    claw.openClaw();
                    follower.followPath(back);
                    orientation.moveNormal();
                    panning.setTargetPos(pick);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTime() > 500) {
                    claw.closeClaw();
                    panningServo.moveSpecific(panningPick);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTime() > 500) {
                    claw.openClaw();
                    setPathState(13);
                }
                break;
            case 13:
                if (follower.atParametricEnd() || follower.isRobotStuck()) {
                    follower.breakFollowing();
                    claw.closeClaw();
                    sleep(100);
                    panningServo.moveSpecific(panningScore);
                    orientation.moveOpposite();
                    follower.followPath(hang);
                    panning.setTargetPos(score);
                    slides.setTargetPos(slideScore);
                    setPathState(14);
                }
                break;
            case 14:
                if (follower.atParametricEnd()) {
                    slides.setTargetPos(0);
                    while (slides.getCurrentPos() > 650) {
                        slides.updateSlide();
                        slides.updatePower();
                        follower.update();
                    }
                    claw.openClaw();
                    follower.followPath(back);
                    orientation.moveNormal();
                    panning.setTargetPos(pick);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTime() > 500) {
                    claw.closeClaw();
                    panningServo.moveSpecific(panningPick);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTime() > 500) {
                    claw.openClaw();
                    setPathState(17);
                }
                break;
            case 17:
                if (follower.atParametricEnd() || follower.isRobotStuck()) {
                    follower.breakFollowing();
                    claw.closeClaw();
                    sleep(100);
                    panningServo.moveSpecific(panningScore);
                    orientation.moveOpposite();
                    follower.followPath(hang);
                    panning.setTargetPos(score);
                    slides.setTargetPos(slideScore);
                    setPathState(18);
                }
                break;
            case 18:
                if (follower.atParametricEnd()) {
                    slides.setTargetPos(0);
                    while (slides.getCurrentPos() > 550) {
                        slides.updateSlide();
                        slides.updatePower();
                        follower.update();
                    }
                    claw.openClaw();
                    follower.followPath(back);
                    orientation.moveNormal();
                    panning.setTargetPos(pick);
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTimer.getElapsedTime() > 500) {
                    claw.closeClaw();
                    panningServo.moveSpecific(panningPick);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTimer.getElapsedTime() > 500) {
                    claw.openClaw();
                    setPathState(21);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        hangFirst = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(134.937, 78.948, Point.CARTESIAN),
                                new Point(108.151, 78.747, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(6).build();

        pushTwo = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(108.151, 78.747, Point.CARTESIAN),
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
                                new Point(87.206, 137.152, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        pushLast = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(87.206, 137.152, Point.CARTESIAN),
                                new Point(129.902, 137.152, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(4).build();
        longHang = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(129.902, 137.152, Point.CARTESIAN),
                                new Point(121.242, 44.509, Point.CARTESIAN),
                                new Point(106.943, 76.129, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(2.5).build();
        longBack = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(106.943, 76.129, Point.CARTESIAN),
                                new Point(117.415, 101.706, Point.CARTESIAN),
                                new Point(131.513, 108.755, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(1.5).build();
        hang = follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(131.513, 108.755, Point.CARTESIAN),
                                new Point(118.624, 48.738, Point.CARTESIAN),
                                new Point(106.943, 73.108, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(2.5).build();
        back = follower.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(106.943, 73.108, Point.CARTESIAN),
                                new Point(116.006, 101.303, Point.CARTESIAN),
                                new Point(131.513, 108.957, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(1.5).build();
        backFast = follower.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(107.748, 74.115, Point.CARTESIAN),
                                new Point(116.006, 101.303, Point.CARTESIAN),
                                new Point(131.513, 108.957, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(10).build();


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
        dSensor =  hardwareMap.get(DistanceSensor.class, "distance");
        light = new Light(hardwareMap);

        // Set up follower
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        buildPaths();

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        // Place parts in initial positions
        claw.closeClaw(); // Closed claw
        pitching.moveUp(); // Pitching UP
        orientation.moveNormal(); // Orientation at normal pos

        panning.setTargetPos(370);
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
}
