package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
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
public class SUBMERSIBLE6 extends LinearOpMode {
    public PIDFPanning panning;
    public PIDFSlide slides;
    public Claw claw;
    public Orientation orientation;
    public Pitching pitching;
    public PanningServo panningServo;
    public Light light;

    private Timer pathTimer;
    int score = 1700;
    int pick = 0;

    double panningScore = 0.9;
    double panningPick = 0.5;

    int slideScore = 850;
    int slideStart = 1100;

    private Follower follower;
    private final Pose startPose = new Pose(136, 80, Math.toRadians(180));

    private PathChain line1, line2, line3, line4, line5,
                      line6, line7, line8, line9, line10, entireThing;

    public static PathBuilder builder = new PathBuilder();

    private int pathState;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(entireThing);
                setPathState(1);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        line1 = builder
                .addPath(
                        new BezierLine(
                                new Point(136.000, 80.000, Point.CARTESIAN),
                                new Point(115.000, 79.739, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        line2 = builder
                .addPath(
                        new BezierCurve(
                                new Point(115.000, 79.739, Point.CARTESIAN),
                                new Point(117.159, 98.547, Point.CARTESIAN),
                                new Point(110.302, 108.147, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .build();

        line3 = builder
                .addPath(
                        new BezierLine(
                                new Point(110.302, 108.147, Point.CARTESIAN),
                                new Point(119.510, 112.457, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(45))
                .build();

        line4 = builder
                .addPath(
                        new BezierLine(
                                new Point(119.510, 112.457, Point.CARTESIAN),
                                new Point(109.714, 118.531, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(130))
                .build();

        line5 = builder
                .addPath(
                        new BezierLine(
                                new Point(109.714, 118.531, Point.CARTESIAN),
                                new Point(118.139, 122.057, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(45))
                .build();

        line6 = builder
                .addPath(
                        new BezierLine(
                                new Point(118.139, 122.057, Point.CARTESIAN),
                                new Point(114.612, 125.584, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(130))
                .build();

        line7 = builder
                .addPath(
                        new BezierLine(
                                new Point(114.612, 125.584, Point.CARTESIAN),
                                new Point(117.747, 124.212, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(45))
                .build();

        line8 = builder
                .addPath(
                        new BezierCurve(
                                new Point(117.747, 124.212, Point.CARTESIAN),
                                new Point(116.180, 113.633, Point.CARTESIAN),
                                new Point(133.812, 115.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        line9 = builder
                .addPath(
                        new BezierCurve(
                                new Point(133.812, 115.000, Point.CARTESIAN),
                                new Point(124.604, 76.212, Point.CARTESIAN),
                                new Point(112.457, 86.400, Point.CARTESIAN),
                                new Point(113.241, 81.306, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line10 = builder
                .addPath(
                        new BezierCurve(
                                new Point(113.241, 81.306, Point.CARTESIAN),
                                new Point(122.057, 68.571, Point.CARTESIAN),
                                new Point(109.910, 119.706, Point.CARTESIAN),
                                new Point(133.812, 115.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        entireThing = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(136.000, 80.000, Point.CARTESIAN),
                                new Point(115.000, 79.739, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierCurve(
                                new Point(115.000, 79.739, Point.CARTESIAN),
                                new Point(117.159, 98.547, Point.CARTESIAN),
                                new Point(110.302, 108.147, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .addPath(
                        new BezierLine(
                                new Point(110.302, 108.147, Point.CARTESIAN),
                                new Point(119.510, 112.457, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(45))
                .addPath(
                        new BezierLine(
                                new Point(119.510, 112.457, Point.CARTESIAN),
                                new Point(109.714, 118.531, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(130))
                .addPath(
                        new BezierLine(
                                new Point(109.714, 118.531, Point.CARTESIAN),
                                new Point(118.139, 122.057, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(45))
                .addPath(
                        new BezierLine(
                                new Point(118.139, 122.057, Point.CARTESIAN),
                                new Point(114.612, 125.584, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(130))
                .addPath(
                        new BezierLine(
                                new Point(114.612, 125.584, Point.CARTESIAN),
                                new Point(117.747, 124.212, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(45))
                .addPath(
                        new BezierCurve(
                                new Point(117.747, 124.212, Point.CARTESIAN),
                                new Point(116.180, 113.633, Point.CARTESIAN),
                                new Point(133.812, 115.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(133.812, 115.000, Point.CARTESIAN),
                                new Point(124.604, 76.212, Point.CARTESIAN),
                                new Point(112.457, 86.400, Point.CARTESIAN),
                                new Point(113.241, 81.306, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(113.241, 81.306, Point.CARTESIAN),
                                new Point(122.057, 68.571, Point.CARTESIAN),
                                new Point(109.910, 119.706, Point.CARTESIAN),
                                new Point(133.812, 115.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
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
        light = new Light(hardwareMap);

        // Set up follower
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        buildPaths();

        pathTimer = new Timer();

        // Place parts in initial positions
        claw.openClaw(); // Closed claw
        pitching.moveUp(); // Pitching UP
        orientation.moveNormal(); // Orientation at normal pos

        panning.setTargetPos(370);
        slides.setTargetPos(0);
        light.goToWhite();

        // Set path state to initial
        setPathState(0);

        while (opModeInInit()) {
            slides.updateSlide();
            panning.updatePanning();
            slides.updatePower();
        }

        waitForStart();
        follower.setPose(startPose);
        pathTimer.resetTimer();

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
