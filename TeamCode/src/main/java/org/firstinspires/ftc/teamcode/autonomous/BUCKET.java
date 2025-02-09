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
import org.firstinspires.ftc.teamcode.vision.Vision;

@Autonomous
public class BUCKET extends LinearOpMode {
    public PIDFPanning panning;
    public PIDFSlide slides;
    public Claw claw;
    public Orientation orientation;
    public Pitching pitching;
    public PanningServo panningServo;
    public DistanceSensor dSensor;
    public Light light;

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
                panning.setTargetPos(1700);
                while (panning.getCurrentPos() < 1500) {
                    panning.updatePanning();
                    follower.update();
                }
                slides.setTargetPos(2300);
                //slides.setTargetPos(2300);
                setPathState(2);
                break;
            case 2:
                if (slides.getCurrentPos() > 2100) {
                    // Move servos
                    panningServo.moveSpecific(0.45);
                    setPathState(123);
                }
                break;
            case 123:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
                    claw.openClaw();
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 1000) {
                    panningServo.moveDown();
                    sleep(300);
                    follower.followPath(pickup1, true);
                    slides.setTargetPos(700);
                    while (slides.getCurrentPos() > 800) {
                        slides.updateSlide();
                        slides.updatePower();
                        follower.update();
                    }
                    panning.setTargetPos(0);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    // Add servo movements
                    pitching.moveDown();
                    sleep(250);
                    claw.closeClaw();
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTime() > 200) {
                    pitching.moveUp();
                    follower.followPath(bucket1);
                    slides.setTargetPos(2300);
                    panning.setTargetPos(1700);
                    setPathState(7);
                }
                break;
            case 7:
                if (slides.getCurrentPos() > 2100) {
                    // Move servos
                    panningServo.moveSpecific(0.45);
                    setPathState(456);
                }
                break;
            case 456:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
                    claw.openClaw();
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTime() > 1000) {
                    panningServo.moveDown();
                    sleep(300);
                    follower.followPath(pickup2, true);
                    slides.setTargetPos(700);
                    while (slides.getCurrentPos() > 800) {
                        slides.updateSlide();
                        slides.updatePower();
                        follower.update();
                    }
                    panning.setTargetPos(0);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    // Add servo movements
                    pitching.moveDown();
                    sleep(250);
                    claw.closeClaw();
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTime() > 200) {
                    pitching.moveUp();
                    follower.followPath(bucket2);
                    panning.setTargetPos(1700);
                    while (panning.getCurrentPos() < 1500) {
                        panning.updatePanning();
                        follower.update();
                    }
                    slides.setTargetPos(2300);
                }
                break;
            case 12:
                if (slides.getCurrentPos() > 2100) {
                    // Move servos
                    panningServo.moveSpecific(0.45);
                    sleep(500);
                    claw.openClaw();
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTime() > 1000) {
                    panningServo.moveDown();
                    sleep(300);
                    follower.followPath(pickup3, true);
                    slides.setTargetPos(700);
                    while (slides.getCurrentPos() > 800) {
                        slides.updateSlide();
                        slides.updatePower();
                        follower.update();
                    }
                    panning.setTargetPos(0);
                    setPathState(333);
                }
                break;
            case 14:
                    if (!follower.isBusy()) {
                        // Add servo movements
                        pitching.moveDown();
                        sleep(250);
                        claw.closeClaw();
                        setPathState(15);
                    }
                break;
            case 15:
                if (pathTimer.getElapsedTime() > 600) {
                    follower.followPath(bucket3);
                    panning.setTargetPos(1700);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTime() > 1000) {
                    slides.setTargetPos(1700);
                    setPathState(17);
                }
            case 17:
                if (slides.getCurrentPos() > 2100) {
                    panningServo.moveSpecific(0.45);
                    sleep(500);
                    claw.openClaw();
                    setPathState(18);

                    //setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(sub1, true);
                    slides.setTargetPos(0);
                    panning.setTargetPos(0);
                    while (slides.getCurrentPos() > 800) {
                        slides.updateSlide();
                        slides.updatePower();
                        follower.update();
                    }

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
                        new BezierCurve(
                                new Point(136.621, 33.317, Point.CARTESIAN),
                                new Point(123.876, 33.540, Point.CARTESIAN),
                                new Point(125.503, 17.304, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135)).build();
        pickup1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(121.863, 20.571, Point.CARTESIAN),
                                new Point(113.790, 16.917, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(160)).build();
        bucket1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(116.006, 14.702, Point.CARTESIAN),
                                new Point(121.863, 20.571, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(135)).build();
        pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(120.839, 19.133, Point.CARTESIAN),
                                new Point(115.956, 12.530, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(193))
                .build();
        bucket2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(115.956, 12.530, Point.CARTESIAN),
                                new Point(119.027, 17.320, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(193), Math.toRadians(135))
                .build();
        pickup3 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(121.863, 20.571, Point.CARTESIAN),
                                new Point(115.804, 4.834, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(200)).build();
        bucket3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(114.037, 12.075, Point.CARTESIAN),
                                new Point(121.863, 20.571, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(135)).build();
        sub1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(119.429, 17.723, Point.CARTESIAN),
                                new Point(70.000, 10.000, Point.CARTESIAN),
                                new Point(72.199, 47.934, Point.CARTESIAN)
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
        dSensor =  hardwareMap.get(DistanceSensor.class, "distance");
        light = new Light(hardwareMap);

        // Set up follower
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.5);
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
        }

        waitForStart();
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
