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
import org.firstinspires.ftc.teamcode.parts.Orientation;
import org.firstinspires.ftc.teamcode.parts.PIDFPanning;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.PanningServo;
import org.firstinspires.ftc.teamcode.parts.Pitching;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Autonomous
public class BUCKETPATHS extends LinearOpMode {
    public PIDFPanning panning;
    public PIDFSlide slides;
    public Claw claw;
    public Orientation orientation;
    public Pitching pitching;
    public PanningServo panningServo;
    //public DistanceSensor dSensor;
    //public Light light;

    private Timer pathTimer, opmodeTimer;

    private Vision vision;

    private Follower follower;
    private final Pose startPose = new Pose(136.621, 33.317, Math.toRadians(90));

    private PathChain bucketInitial, pickup1, bucket1, pickup2, bucket2, pickup3, bucket3, sub1, bucket4, align, entireThing;

    private int pathState;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(bucketInitial, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(pickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(bucket1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(pickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(bucket2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(pickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(bucket3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(sub1, true);
                    setPathState(234);
                }
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        bucketInitial = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(136.621, 33.317, Point.CARTESIAN),
                                new Point(123.876, 33.540, Point.CARTESIAN),
                                new Point(124.263, 19.133, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135)).build();
        pickup1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(124.263, 19.133, Point.CARTESIAN),
                                new Point(115.401, 12.084, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(160)).build();
        bucket1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(115.401, 12.084, Point.CARTESIAN),
                                new Point(122.853, 16.917, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(135)).build();
        pickup2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(122.853, 16.917, Point.CARTESIAN),
                                new Point(116.207, 6.042, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
        bucket2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(116.207, 6.042, Point.CARTESIAN),
                                new Point(120.638, 14.903, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
        pickup3 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(120.638, 14.903, Point.CARTESIAN),
                                new Point(98.685, 16.716, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270)).build();
        bucket3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(98.685, 16.716, Point.CARTESIAN),
                                new Point(123.055, 14.299, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135)).build();
        sub1 = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(123.055, 14.299, Point.CARTESIAN),
                                new Point(83.580, 22.557, Point.CARTESIAN),
                                new Point(84.386, 46.724, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
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
        //dSensor =  hardwareMap.get(DistanceSensor.class, "distance");
        //light = new Light(hardwareMap);

        // Set up follower
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.75);
        buildPaths();

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        // Place parts in initial positions
        claw.closeClaw(); // Closed claw
        pitching.moveUp(); // Pitching UP
        orientation.moveNormal(); // Orientation at normal pos

        panning.setTargetPos(0);
        slides.setTargetPos(0);
        //light.goToBlue();

        // Set path state to initial
        setPathState(0);

        while (opModeInInit()) {
            slides.updateSlide();
            panning.updatePanning();
            slides.updatePower();
        }

        waitForStart();
        panningServo.moveDown();
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
            telemetry.update();
        }
    }
}
