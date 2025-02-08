//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//
//import org.firstinspires.ftc.teamcode.parts.Claw;
//import org.firstinspires.ftc.teamcode.parts.Orientation;
//import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
//import org.firstinspires.ftc.teamcode.parts.Panning;
//import org.firstinspires.ftc.teamcode.parts.PanningServo;
//import org.firstinspires.ftc.teamcode.parts.Pitching;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
//
//@Autonomous
//public class push_submersible extends LinearOpMode {
//    // Parts
//    public Panning panning;
//    public Claw claw;
//    public Orientation orientation;
//    public Pitching pitching;
//    public PanningServo panningServo;
//    public PIDFSlide slides;
//    public DistanceSensor dSensor;
//
//    // Follower stuff
//    private Follower follower;
//    private Timer pathTimer, opmodeTimer;
//    private int pathState;
//
//    // Path chains to be used in buildPaths()
//    private final Pose startPose = new Pose(134.888583218707, 78.10639605599073, 0); // line 1
//    private PathChain hangFirst, pushTwo, pushLast, goToHang1, goBack1, goToHang2, goBack2, goToHang3, entireThing;
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    public void movePanServoScore() { panningServo.moveSpecific(0.1); }
//    public void movePanServoReady() {panningServo.moveSpecific(0.45);}
//
//    public void buildPaths() {
//        hangFirst = follower.pathBuilder() // Line 1
//                .addPath(
//                        // Line 1
//                        new BezierLine(
//                                new Point(134.400, 80.595, Point.CARTESIAN),
//                                new Point(111.405, 78.363, Point.CARTESIAN)
//                        )
//                )
//                .setZeroPowerAccelerationMultiplier(3.8)
//                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
//
//
//        pushTwo = follower.pathBuilder() // Lines 2 - 6
//                .addPath(
//                // Line 2
//                new BezierCurve(
//                        new Point(111.405, 78.363, Point.CARTESIAN),
//                        new Point(119.219, 112.298, Point.CARTESIAN),
//                        new Point(96.000, 100.688, Point.CARTESIAN),
//                        new Point(80.819, 115.870, Point.CARTESIAN)
//                )
//        )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 3
//                        new BezierLine(
//                                new Point(80.819, 115.870, Point.CARTESIAN),
//                                new Point(113.414, 115.647, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 4
//                        new BezierCurve(
//                                new Point(113.414, 115.647, Point.CARTESIAN),
//                                new Point(83.051, 114.530, Point.CARTESIAN),
//                                new Point(83.051, 125.693, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 5
//                        new BezierLine(
//                                new Point(83.051, 125.693, Point.CARTESIAN),
//                                new Point(127.926, 125.916, Point.CARTESIAN)
//                        )
//                )
//                .setZeroPowerAccelerationMultiplier(3.5)
//                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
//
//
//        goToHang1 = follower.pathBuilder() // Lines 8-9
//                .addPath(
//                        // Line 6
//                        new BezierCurve(
//                                new Point(127.926, 125.916, Point.CARTESIAN),
//                                new Point(107.163, 114.753, Point.CARTESIAN),
//                                new Point(139.981, 78.586, Point.CARTESIAN),
//                                new Point(111.628, 72.781, Point.CARTESIAN)
//                        )
//                )
//                .setZeroPowerAccelerationMultiplier(4.8)
//                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
//
//
//        goBack1 = follower.pathBuilder()
//                .addPath(
//                        // Line 7
//                        new BezierCurve(
//                                new Point(111.628, 72.781, Point.CARTESIAN),
//                                new Point(128.595, 70.772, Point.CARTESIAN),
//                                new Point(94.437, 115.200, Point.CARTESIAN),
//                                new Point(128.149, 116.540, Point.CARTESIAN)
//                        )
//                )
//                .setZeroPowerAccelerationMultiplier(2)
//                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
//
//
//        goToHang2 = follower.pathBuilder()
//                .addPath(
//                        // Line 8
//                        new BezierCurve(
//                                new Point(128.149, 116.540, Point.CARTESIAN),
//                                new Point(129.265, 73.674, Point.CARTESIAN),
//                                new Point(111.851, 75.907, Point.CARTESIAN)
//                        )
//                )
//                .setZeroPowerAccelerationMultiplier(4.8)
//                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
//        goBack2 = follower.pathBuilder()
//                .addPath(
//                        // Line 9
//                        new BezierCurve(
//                                new Point(111.851, 75.907, Point.CARTESIAN),
//                                new Point(125.916, 79.256, Point.CARTESIAN),
//                                new Point(97.340, 114.753, Point.CARTESIAN),
//                                new Point(127.926, 116.986, Point.CARTESIAN)
//                        )
//                )
//                .setZeroPowerAccelerationMultiplier(2)
//                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
//        goToHang3 = follower.pathBuilder()
//                .addPath(
//                        // Line 10
//                        new BezierCurve(
//                                new Point(127.926, 116.986, Point.CARTESIAN),
//                                new Point(130.828, 70.326, Point.CARTESIAN),
//                                new Point(111.851, 68.986, Point.CARTESIAN)
//                        )
//                )
//                .setZeroPowerAccelerationMultiplier(4.8)
//                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
//
//
//        entireThing = follower.pathBuilder()
//                .addPath(
//                        // Line 1
//                        new BezierLine(
//                                new Point(134.889, 78.106, Point.CARTESIAN),
//                                new Point(105.772, 77.137, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 2
//                        new BezierCurve(
//                                new Point(105.772, 77.137, Point.CARTESIAN),
//                                new Point(138.850, 124.038, Point.CARTESIAN),
//                                new Point(71.109, 98.068, Point.CARTESIAN),
//                                new Point(86.162, 118.999, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 3
//                        new BezierLine(
//                                new Point(86.162, 118.999, Point.CARTESIAN),
//                                new Point(124.193, 121.519, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 4
//                        new BezierCurve(
//                                new Point(124.193, 121.519, Point.CARTESIAN),
//                                new Point(65.959, 120.162, Point.CARTESIAN),
//                                new Point(87.549, 130.821, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 5
//                        new BezierLine(
//                                new Point(87.549, 130.821, Point.CARTESIAN),
//                                new Point(123.994, 133.535, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 6
//                        new BezierCurve(
//                                new Point(123.994, 133.535, Point.CARTESIAN),
//                                new Point(63.978, 125.782, Point.CARTESIAN),
//                                new Point(86.162, 137.411, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 7
//                        new BezierLine(
//                                new Point(86.162, 137.411, Point.CARTESIAN),
//                                new Point(124.391, 136.635, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 8
//                        new BezierCurve(
//                                new Point(124.391, 136.635, Point.CARTESIAN),
//                                new Point(131.323, 66.091, Point.CARTESIAN),
//                                new Point(105.970, 69.385, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 9
//                        new BezierCurve(
//                                new Point(107.386, 72.335, Point.CARTESIAN),
//                                new Point(112.298, 112.074, Point.CARTESIAN),
//                                new Point(131.944, 114.084, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Get things from hardware map
//        slides = new PIDFSlide(hardwareMap);
//        panning = new Panning(hardwareMap);
//        claw = new Claw(hardwareMap);
//        pitching = new Pitching(hardwareMap);
//        orientation = new Orientation(hardwareMap);
//        panningServo = new PanningServo(hardwareMap);
//        dSensor =  hardwareMap.get(DistanceSensor.class, "distance");
//
//        // Make timers
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//
//        // Set up follower
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//        buildPaths();
//
//        // Place parts in initial positions
//        claw.closeClaw(); // Closed claw
//        panningServo.moveUp(); // Panning servo UP
//        pitching.moveDown(); // Pitching UP
//        orientation.moveNormal(); // Orientation at normal pos
//        setPathState(0);
//
//        waitForStart();
//        opmodeTimer.resetTimer();
//
//        while (opModeIsActive()) {
//            autonomousPathUpdate();
//            follower.update();
//            slides.updateSlide();
//            slides.updatePower();
//
//            // Feedback to Driver Hub
//            telemetry.addData("path state", pathState);
//            telemetry.addData("x", follower.getPose().getX());
//            telemetry.addData("y", follower.getPose().getY());
//            telemetry.addData("heading", follower.getPose().getHeading());
//            telemetry.addData("T value: ", follower.getCurrentTValue());
//            telemetry.update();
//        }
//    }
//}
