package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
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
import org.firstinspires.ftc.teamcode.parts.Sweeping;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous
public class Bucket extends LinearOpMode {
    public PIDFPanning panning;
    public PIDFSlide slides;
    public Claw claw;
    public Orientation orientation;
    public Pitching pitching;
    public PanningServo panningServo;
    public Sweeping sweeping;
    public Light light;

    private Timer pathTimer, opmodeTimer, waitTimer;
    int score = 1800;
    int pick = 0;

    double panningScore = 0.8;

    private boolean updatePanning = true;

    int slideScore = 1360;

    private Follower follower;
    private final Pose startPose = new Pose(140, 38, Math.toRadians(90));

    private PathChain line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line69;

    private int pathState;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(line1, true);
                panningServo.moveDown();
                panning.setTargetPos(1800);
                while (panning.getCurrentPos() < 1000) {
                    updateSystems();
                }
                slides.setTargetPos(slideScore);
                while (slides.getCurrentPos() < slideScore - 100) {
                    updateSystems();
                }
                panningServo.moveUp();
                waitTimer(250);
                claw.openClaw();
                waitTimer(250);
                panningServo.moveDown();
                waitTimer(250);
                slides.setTargetPos(0);
                orientation.moveNormal();
                setPathState(1);
                break;
            case 1:
                follower.followPath(line2, true);
                waitTimer(500);
                panning.setTargetPos(0);
                waitTimer(500);
                slides.setTargetPos(625);
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) {
                    updatePanning = false;
                    pitching.moveDown();
                    waitTimer(250);
                    claw.closeClaw();
                    waitTimer(250);
                    pitching.moveUp();
                    follower.followPath(line5, true);
                    updatePanning = true;
                    waitTimer(500);
                    slides.setTargetPos(0);
                    panningServo.moveDown();
                    panning.setTargetPos(1800);
                    while (panning.getCurrentPos() < 1000) {
                        updateSystems();
                    }
                    slides.setTargetPos(slideScore);
                    while (slides.getCurrentPos() < slideScore - 100) {
                        updateSystems();
                    }
                    panningServo.moveUp();
                    waitTimer(250);
                    claw.openClaw();
                    waitTimer(250);
                    panningServo.moveDown();
                    waitTimer(250);
                    slides.setTargetPos(0);
                    orientation.moveNormal();
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(line4, true);
                waitTimer(500);
                panning.setTargetPos(0);
                waitTimer(500);
                slides.setTargetPos(625);
                setPathState(4);
                break;
            case 4:
                if (!follower.isBusy()) {
                    pitching.moveDown();
                    waitTimer(250);
                    claw.closeClaw();
                    waitTimer(250);
                    pitching.moveUp();
                    follower.followPath(line5, true);
                    waitTimer(500);
                    slides.setTargetPos(0);
                    panningServo.moveSpecific(0.5);
                    panning.setTargetPos(1800);
                    while (panning.getCurrentPos() < 1000) {
                        updateSystems();
                    }
                    slides.setTargetPos(slideScore);
                    while (slides.getCurrentPos() < slideScore - 100) {
                        updateSystems();
                    }
                    panningServo.moveUp();
                    waitTimer(250);
                    claw.openClaw();
                    waitTimer(250);
                    panningServo.moveDown();
                    waitTimer(250);
                    slides.setTargetPos(0);
                    orientation.moveNormal();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(line6, true);
                    waitTimer(500);
                    panning.setTargetPos(0);
                    waitTimer(500);
                    slides.setTargetPos(575);
                    orientation.moveSpecific(0.25);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    updatePanning = false;
                    pitching.moveDown();
                    waitTimer(250);
                    claw.closeClaw();
                    waitTimer(250);
                    pitching.moveUp();
                    follower.followPath(line7, true);
                    updatePanning = true;
                    waitTimer(500);
                    slides.setTargetPos(0);
                    panningServo.moveDown();
                    panning.setTargetPos(1800);
                    while (panning.getCurrentPos() < 1000) {
                        updateSystems();
                    }
                    slides.setTargetPos(slideScore);
                    while (slides.getCurrentPos() < slideScore - 100) {
                        updateSystems();
                    }
                    panningServo.moveUp();
                    waitTimer(250);
                    claw.openClaw();
                    waitTimer(250);
                    panningServo.moveDown();
                    waitTimer(250);
                    slides.setTargetPos(0);
                    orientation.moveNormal();
                    setPathState(69);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        line1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(140, 38, Point.CARTESIAN),
                                new Point(126, 18, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(750)
                .build();

        line2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(126, 18, Point.CARTESIAN),
                                new Point(115, 21, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(750)
                .build();

        line3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(115, 21, Point.CARTESIAN),
                                new Point(124, 22, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(750)
                .build();

        line4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(124, 22, Point.CARTESIAN),
                                new Point(115, 12, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(750)
                .build();

        line5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(115, 12, Point.CARTESIAN),
                                new Point(124, 22, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(750)
                .build();

        line6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(124, 22, Point.CARTESIAN),
                                new Point(112, 8, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(230))
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(750)
                .build();

        line7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(112.000, 8, Point.CARTESIAN),
                                new Point(124, 22, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(135))
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(750)
                .build();
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
        sweeping = new Sweeping(hardwareMap);
        light = new Light(hardwareMap);

        // Set up follower
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setMaxPower(0.75);
        buildPaths();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        waitTimer = new Timer();

        // Place parts in initial positions
        pitching.moveUp();
        orientation.moveSpecific(0.5);
        panningServo.moveUp();
        sweeping.sweeperUp();
        claw.closeClaw();
        light.goToWhite();

        panning.setTargetPos(0);
        slides.setTargetPos(0);

        // Set path state to initial
        setPathState(0);

        while (opModeInInit()) {
            slides.updateSlide();
            panning.updatePanning();
        }

        waitForStart();
        follower.setPose(startPose);
        opmodeTimer.resetTimer();
        waitTimer.resetTimer();

        while (opModeIsActive()) {
            autonomousPathUpdate();
            follower.update();
            if (updatePanning) {
                panning.updatePanning();
            }
            panning.update();
            if (panning.getCurrentPos() < 0) {
                panning.resetEncoder();
            }
            slides.updateSlide();

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
        if (updatePanning) {
            panning.updatePanning();
        }
        panning.update();
        if (panning.getCurrentPos() < 0) {
            panning.resetEncoder();
        }
        slides.updateSlide();
        follower.update();
    }

    public void updateSystems() {
        if (updatePanning) {
            panning.updatePanning();
        }
        panning.update();
        if (panning.getCurrentPos() < 0) {
            panning.resetEncoder();
        }
        slides.updateSlide();
        follower.update();
    }

    public void waitTimer(int timeMs) {
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() < timeMs) {updateSystems();}
    }
}
