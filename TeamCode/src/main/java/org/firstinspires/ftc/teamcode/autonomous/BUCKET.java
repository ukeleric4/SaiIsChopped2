package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private Limelight3A limelight;
    public Light light;

    LLResult result;
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    double offsetX;
    double offsetY;
    double[] pythonResults;
    double uhorientation;
    double power = 0;

    Timer followerTime;
    Timer waitTimer;
    int targetPosition = 1250;
    int slideUp = 2300;

    double servoOuttake = 0.6;
    double servoIntake = 0.4;

    private Timer pathTimer, opmodeTimer;

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
                slides.setTargetPos(slideUp);
                setPathState(1);
                break;
            case 1:
                if (slides.getCurrentPos() > 2000) {
                    // Move servos
                    panningServo.moveSpecific(servoOuttake);
                    setPathState(2);
                }
                break;
            case 2:
                if (slides.getCurrentPos() > (slideUp - 80) && !follower.isBusy()) {
                    claw.openClaw();
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 350) {
                    panningServo.moveSpecific(servoIntake);
                    waitTimer(150);
                    follower.followPath(pickup1, true);
                    slides.setTargetPos(0);
                    while (slides.getCurrentPos() > 100) {
                        slides.updateSlide();
                        slides.updatePower();
                        panning.updatePanning();
                        follower.update();
                    }
                    panning.setTargetPos(0);
                    while (panning.getCurrentPos() > 200) {
                        slides.updateSlide();
                        slides.updatePower();
                        panning.updatePanning();
                        follower.update();
                    }
                    slides.setTargetPos(850);
                    while (slides.getCurrentPos() < 550) {
                        slides.updateSlide();
                        slides.updatePower();
                        panning.updatePanning();
                        follower.update();
                    }
                    setPathState(4);
                }
                break;
            case 4:
                if (slides.getCurrentPos() > 840) {
                    result = limelight.getLatestResult();
                    updateLimelight();
                    follower.followPath(bucket1);
                    panning.setTargetPos(1700);
                    while (panning.getCurrentPos() < 1500) {
                        updateImportant();
                    }
                    slides.setTargetPos(slideUp);
                    setPathState(5);
                }
                break;
            case 5:
                if (slides.getCurrentPos() > 2000) {
                    panningServo.moveSpecific(servoOuttake);
                    setPathState(6);
                }
                break;
            case 6:
                if (slides.getCurrentPos() > (slideUp - 80) && !follower.isBusy()) {
                    claw.openClaw();
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTime() > 350) {
                    panningServo.moveSpecific(servoIntake);
                    waitTimer(150);
                    follower.followPath(pickup2, true);
                    slides.setTargetPos(0);
                    while (slides.getCurrentPos() > 100) {
                        slides.updateSlide();
                        slides.updatePower();
                        panning.updatePanning();
                        follower.update();
                    }
                    panning.setTargetPos(0);
                    while (panning.getCurrentPos() > 200) {
                        slides.updateSlide();
                        slides.updatePower();
                        panning.updatePanning();
                        follower.update();
                    }
                    slides.setTargetPos(850);
                    while (slides.getCurrentPos() < 550) {
                        slides.updateSlide();
                        slides.updatePower();
                        panning.updatePanning();
                        follower.update();
                    }
                    setPathState(8);
                }
                break;
            case 8:
                if (slides.getCurrentPos() > 840) {
                    result = limelight.getLatestResult();
                    updateLimelight();
                    follower.followPath(bucket2);
                    panning.setTargetPos(1700);
                    while (panning.getCurrentPos() < 1500) {
                        updateImportant();
                    }
                    slides.setTargetPos(slideUp);
                    setPathState(9);
                }
                break;
            case 9:
                if (slides.getCurrentPos() > 2000) {
                    panningServo.moveSpecific(servoOuttake);
                    setPathState(10);
                }
                break;
            case 10:
                if (slides.getCurrentPos() > (slideUp - 80) && !follower.isBusy()) {
                    claw.openClaw();
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTime() > 350) {
                    panningServo.moveSpecific(servoIntake);
                    waitTimer(150);
                    follower.setMaxPower(0.75);
                    follower.followPath(pickup3, true);
                    slides.setTargetPos(0);
                    while (slides.getCurrentPos() > 100) {
                        slides.updateSlide();
                        slides.updatePower();
                        panning.updatePanning();
                        follower.update();
                    }
                    panning.setTargetPos(0);
                    while (panning.getCurrentPos() > 200) {
                        slides.updateSlide();
                        slides.updatePower();
                        panning.updatePanning();
                        follower.update();
                    }
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    slides.setTargetPos(850);
                    while (slides.getCurrentPos() < 840) {
                        slides.updateSlide();
                        slides.updatePower();
                        panning.updatePanning();
                        follower.update();
                    }
                    result = limelight.getLatestResult();
                    updateLimelight();
                    follower.followPath(bucket3);
                    panning.setTargetPos(1700);
                    while (panning.getCurrentPos() < 1500) {
                        updateImportant();
                    }
                    slides.setTargetPos(slideUp);
                    setPathState(13);
                }
                break;
            case 13:
                if (slides.getCurrentPos() > 2000) {
                    panningServo.moveSpecific(servoOuttake);
                    setPathState(14);
                }
                break;
            case 14:
                if (slides.getCurrentPos() > (slideUp - 80) && !follower.isBusy()) {
                    claw.openClaw();
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTime() > 350) {
                    panningServo.moveSpecific(servoIntake);
                    waitTimer(150);
                    follower.setMaxPower(1.0);
                    follower.followPath(sub1, true);
                    slides.setTargetPos(0);
                    while (slides.getCurrentPos() > 100) {
                        slides.updateSlide();
                        slides.updatePower();
                        panning.updatePanning();
                        follower.update();
                    }
                    panning.setTargetPos(0);
                    while (panning.getCurrentPos() > 200) {
                        slides.updateSlide();
                        slides.updatePower();
                        panning.updatePanning();
                        follower.update();
                    }
                    setPathState(999);
                }
                break;
            case 999:
                if (!follower.isBusy()) {
                    slides.setTargetPos(850);
                    setPathState(16);
                }
                break;
            case 16:
                if (slides.getCurrentPos() > 840) {
                    result = limelight.getLatestResult();
                    updateLimelight();
                    follower.followPath(bucket4);
                    panning.setTargetPos(1700);
                    while (panning.getCurrentPos() < 1500) {
                        updateImportant();
                    }
                    setPathState(666);
                }
                break;
            case 666:
                if (follower.getCurrentTValue() > 0.7) {
                    slides.setTargetPos(slideUp);
                }
                break;
            case 17:
                if (slides.getCurrentPos() > 2000) {
                    panningServo.moveSpecific(servoOuttake);
                    setPathState(18);
                }
                break;
            case 18:
                if (slides.getCurrentPos() > (slideUp - 80) && !follower.isBusy()) {
                    claw.openClaw();
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
                        new BezierCurve(
                                new Point(136.621, 33.317, Point.CARTESIAN),
                                new Point(123.876, 33.540, Point.CARTESIAN),
                                new Point(124.263, 19.133, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135)).build();
        pickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(124.263, 19.133, Point.CARTESIAN),
                                new Point(115.401, 12.084, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(160)).build();
        bucket1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(115.401, 12.084, Point.CARTESIAN),
                                new Point(122.853, 16.917, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(135)).build();
        pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(122.853, 16.917, Point.CARTESIAN),
                                new Point(116.207, 6.042, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(193))
                .build();
        bucket2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(116.207, 6.042, Point.CARTESIAN),
                                new Point(121.078, 13.910, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(193), Math.toRadians(135))
                .build();
        pickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(121.078, 13.910, Point.CARTESIAN),
                                new Point(98.743, 20.571, Point.CARTESIAN),
                                new Point(106, 20, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270)).build();
        bucket3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(106, 20, Point.CARTESIAN),
                                new Point(122.057, 14.694, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135)).build();
        sub1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(122.057, 14.694, Point.CARTESIAN),
                                new Point(83.580, 22.557, Point.CARTESIAN),
                                new Point(80.000, 46.724, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();
        bucket4 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(80, 46.724, Point.CARTESIAN),
                                new Point(83.580, 22.557, Point.CARTESIAN),
                                new Point(124.062, 16.313, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        followerTime = new Timer();
        waitTimer = new Timer();
        limelight.pipelineSwitch(3);
        limelight.start();

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

        panning.setTargetPos(0);
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
        waitTimer.resetTimer();
        followerTime.resetTimer();
        opmodeTimer.resetTimer();
        panningServo.moveDown();

        while (opModeIsActive()) {
            autonomousPathUpdate();
            result = limelight.getLatestResult();
            telemetry.addData("Result: ", result);
            follower.update();
            panning.updatePanning();
            slides.updateSlide();
            slides.updatePower();

            if (result != null) {
                telemetry.addData("Result found:", true);
            } else {
                telemetry.addData("Result found:", false);
            }

            // Feedback to Driver Hub
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("T value: ", follower.getCurrentTValue());
            telemetry.update();
        }
    }

    public void updateLimelight() {
        if (result != null) {
            updateResults();
            panningServo.moveSpecific(0.5);
            limelight.pipelineSwitch(3);
            followerTime.resetTimer();
            while (followerTime.getElapsedTime() < 750) {
                updateResults();
                slides.setTargetPos(slides.getCurrentPos() + targetPosition);
                orientation.moveSpecific(uhorientation);
                slides.updateSlide();
                slides.updatePower();
                strafe(power);
            }
                slides.setPower(0);
                strafe(0);
                panningServo.moveDown();
                waitTimer(450);
                pitching.moveDown();
                waitTimer(200);
                claw.closeClaw();
                waitTimer(300);
                pitching.moveUp();
                panningServo.moveSpecific(0.45);
                slides.setTargetPos(0);
        }
    }

    public void strafe(double power) {
        fl.setPower(power);
        br.setPower(power);
        fr.setPower(-power);
        bl.setPower(-power);
    }

    public void updateResults() {
        limelight.pipelineSwitch(3);
        result = limelight.getLatestResult();
        offsetX = result.getTx();
        offsetY = result.getTy();
        if (offsetX < -1 || offsetX > 1) {
            if (offsetX > 6 || offsetX < -6) {
                power = (offsetX) / 50;
            } else {
                power = (offsetX) / 22.5;
            }
        } else {
            power = 0;
        }
        targetPosition = (int) ((-10 + offsetY) * 5);

        pythonResults = result.getPythonOutput();
        uhorientation = pythonResults[6];

        telemetry.addData("offsetX: ", offsetX);
        telemetry.addData("offsetY: ", offsetY);
        telemetry.addData("pos target: ", targetPosition);
        telemetry.addData("power: ", power);
        telemetry.update();
    }

    public void waitTimer(int timeMs) {
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() < timeMs) {}
    }

    public void updateImportant() {
        panning.updatePanning();
        slides.updateSlide();
        slides.updatePower();
        follower.update();
        telemetry.update();
    }
}
