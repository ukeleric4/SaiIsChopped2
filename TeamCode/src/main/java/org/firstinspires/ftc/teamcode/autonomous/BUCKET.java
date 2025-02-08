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
                slides.setTargetPos(2300);
                panning.setTargetPos(1700);
                setPathState(2);
                break;
            case 2:
                if (slides.getCurrentPos() > 2000) {
                    // Move servos
                    panningServo.moveSpecific(0.45);
                    sleep(500);
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
                if (slides.getCurrentPos() > 2000) {
                    // Move servos
                    panningServo.moveSpecific(0.45);
                    sleep(500);
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
                    slides.setTargetPos(2300);
                    panning.setTargetPos(1700);
                    setPathState(11);
                }
                break;
            case 12:
                if (slides.getCurrentPos() > 2000) {
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
                    panning.setTargetPos(0);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    // Add servo movements
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTime() > 600) {
                    follower.followPath(bucket3);
                    slides.setTargetPos(2300);
                    setPathState(16);
                }
                break;
            case 16:
                if (slides.getCurrentPos() > 200) {
                    panning.setTargetPos(1700);
                    setPathState(17);
                }
            case 17:
                if (slides.getCurrentPos() > 2000) {
                    // Move servos

                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(sub1, true);
                    slides.setTargetPos(1000);
                    panning.setTargetPos(0);
                    setPathState(19);
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
                                new Point(121.863, 20.571, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135)).build();
        pickup1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(121.863, 20.571, Point.CARTESIAN),
                                new Point(116.006, 14.702, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(173)).build();
        bucket1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(116.006, 14.702, Point.CARTESIAN),
                                new Point(121.863, 20.571, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(173), Math.toRadians(135)).build();
        pickup2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(121.863, 20.571, Point.CARTESIAN),
                                new Point(116.811, 6.042, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build();
        bucket2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(116.811, 6.042, Point.CARTESIAN),
                                new Point(121.863, 20.571, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)).build();
        pickup3 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(121.863, 20.571, Point.CARTESIAN),
                                new Point(115.804, 4.834, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(215)).build();
        bucket3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(114.037, 12.075, Point.CARTESIAN),
                                new Point(121.863, 20.571, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(215), Math.toRadians(135)).build();
        sub1 = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(121.863, 20.571, Point.CARTESIAN),
                                new Point(88.099, 19.453, Point.CARTESIAN),
                                new Point(84.075, 42.037, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(110)).build();
        bucket4 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(84.075, 42.037, Point.CARTESIAN),
                                new Point(89.665, 18.783, Point.CARTESIAN),
                                new Point(121.863, 20.571, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(135)).build();
        entireThing = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(136.621, 33.317, Point.CARTESIAN),
                                new Point(123.876, 33.540, Point.CARTESIAN),
                                new Point(121.863, 20.571, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(121.863, 20.571, Point.CARTESIAN),
                                new Point(118.509, 21.242, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(173))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(118.509, 21.242, Point.CARTESIAN),
                                new Point(121.863, 20.571, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(173), Math.toRadians(135))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(121.863, 20.571, Point.CARTESIAN),
                                new Point(118.733, 12.522, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(118.733, 12.522, Point.CARTESIAN),
                                new Point(121.863, 20.571, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(121.863, 20.571, Point.CARTESIAN),
                                new Point(114.037, 12.075, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(215))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(114.037, 12.075, Point.CARTESIAN),
                                new Point(121.863, 20.571, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(215), Math.toRadians(135))
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(121.863, 20.571, Point.CARTESIAN),
                                new Point(88.099, 19.453, Point.CARTESIAN),
                                new Point(84.075, 42.037, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(110))
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(84.075, 42.037, Point.CARTESIAN),
                                new Point(89.665, 18.783, Point.CARTESIAN),
                                new Point(121.863, 20.571, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(135)).build();
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
