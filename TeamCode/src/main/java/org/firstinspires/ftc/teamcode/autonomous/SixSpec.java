package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.LiatClaw;
import org.firstinspires.ftc.teamcode.parts.LiatOrientation;
import org.firstinspires.ftc.teamcode.parts.Light;
import org.firstinspires.ftc.teamcode.parts.Orientation;
import org.firstinspires.ftc.teamcode.parts.PIDFPanning;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.PanningServo;
import org.firstinspires.ftc.teamcode.parts.Pitching;
import org.firstinspires.ftc.teamcode.parts.SpecimenArm;
import org.firstinspires.ftc.teamcode.parts.SpecimenArm2;
import org.firstinspires.ftc.teamcode.parts.Sweeping;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous
public class SixSpec extends LinearOpMode {
    //public PIDFPanning panning;
    public PIDFSlide slides;
    public Claw claw;
    public Orientation orientation;
    public Pitching pitching;
    public PanningServo panningServo;
    public Sweeping sweeping;
    private Limelight3A limelight;
    private Light light;
    private LiatOrientation liatOrientation;
    private LiatClaw liatClaw;
    private SpecimenArm2 specimenArm;
    private PIDFPanning panning;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    double offsetX;
    double offsetY;
    double[] pythonResults;
    double uhorientation;
    double power = 0;

    LLResult result;

    Timer followerTime;
    int targetPosition = 500;

    private Timer pathTimer, opmodeTimer, waitTimer;

    double xValue = 0;
    double yValue = 0;

    int slideScore1 = 375;

    private boolean updateSigma = true;
    private boolean sigmaSlide = false;
    private double sigmaOrientation = 0;

    private Follower follower;
    private final Pose startPose = new Pose(132, 80, Math.toRadians(180));
    private Pose pickUpPose;

    private PathChain line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line69, line420, line67, line11, line12, line13, line14;

    private int pathState;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                pitching.moveUp();
                liatOrientation.moveScore();
                panning.setPower(-1);
                panningServo.moveSpecific(0.5);
                claw.openClaw();
                waitTimer(500);
                specimenArm.moveSpecific(0.725);
                slides.setTargetPos(slideScore1);
                panningServo.moveSpecific(0.5);
                follower.followPath(line1, true);
                waitTimer(500);
                panning.setPower(0);
                setPathState(42069);
                break;
            case 42069:
                if (follower.getCurrentTValue() > 0.925 && follower.getTranslationalError().getMagnitude() < 2) {
                    specimenArm.scorePush();
                    follower.followPath(line420);
                    waitTimer(250);
                    panningServo.moveDown();
                    liatClaw.openClaw();
                    setPathState(420);
                }
                break;
            case 420:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 0.5) {
                    waitTimer(500);
                    pitching.moveDown();
                    waitTimer(250);
                    claw.closeClaw();
                    waitTimer(250);
                    pitching.moveUp();
                    slides.setTargetPos(0);
                    panningServo.moveSpecific(0.5);
                    follower.resumePathFollowing();
                    follower.followPath(line69, true);
                    follower.update();
                    pitching.moveUp();
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    specimenArm.pickUp();
                    liatOrientation.movePick();
                    claw.openClaw();
                    follower.followPath(line2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (follower.getCurrentTValue() > 0.75) {
                    sweeping.sweeperDown();
                }
                if (!follower.isBusy()) {
                    follower.followPath(line3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (follower.getCurrentTValue() > 0.8) {
                    sweeping.sweeperUp();
                }
                if (!follower.isBusy()) {
                    follower.followPath(line4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (follower.getCurrentTValue() > 0.75) {
                    sweeping.sweeperDown();
                }
                if (!follower.isBusy()) {
                    follower.followPath(line5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    sweeping.sweeperUp();
                    follower.followPath(line6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (follower.getCurrentTValue() > 0.75) {
                    sweeping.sweeperDown();
                }
                if (!follower.isBusy()) {
                    follower.followPath(line7, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (follower.getCurrentTValue() > 0.75) {
                    sweeping.sweeperUp();
                }
                if (!follower.isBusy()) {
                    follower.followPath(line8, true);
                    setPathState(67);
                }
                break;
            case 67:
                if (follower.atParametricEnd()) {
                    pitching.moveDown();
                    updateSigma = false;
                    sigmaSlide = true;
                    specimenArm.pickUp();
                    follower.followPath(line67, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (follower.getCurrentTValue() > 0.85) {
                    liatClaw.closeClaw();
                }
                if (follower.atParametricEnd() || follower.getVelocity().getMagnitude() < 0.25) {
                    panningServo.moveDown();
                    specimenArm.score();
                    follower.followPath(line9);
                    setPathState(9);
                }
                break;
            case 9:
                if (follower.getCurrentTValue() > 0.05) {
                    specimenArm.score();
                }
                if (follower.getCurrentTValue() > 0.3) {
                    liatOrientation.moveScore();
                    panningServo.moveSpecific(0.5);
                }
                if (follower.getCurrentTValue() > 0.98 && follower.getTranslationalError().getMagnitude() < 2) {
                    liatOrientation.moveSpecific(0.9);
                    specimenArm.scorePush();
                    follower.followPath(line10);
                    setPathState(10);
                }
                break;
            case 10:
                if (follower.getCurrentTValue() > 0.05) {
                    liatClaw.openClaw();
                }
                if (follower.getCurrentTValue() > 0.1) {
                    specimenArm.pickUp();
                }
                if (follower.getCurrentTValue() > 0.25) {
                    liatOrientation.movePick();
                    waitTimer(250);
                }
                if (follower.getCurrentTValue() > 0.9) {
                    liatClaw.closeClaw();
                    follower.followPath(line11);
                    setPathState(11);
                }
                break;
            case 11:
                if (follower.getCurrentTValue() > 0.05) {
                    specimenArm.score();
                }
                if (follower.getCurrentTValue() > 0.3) {
                    liatOrientation.moveScore();
                }
                if (follower.getCurrentTValue() > 0.98 && follower.getTranslationalError().getMagnitude() < 2) {
                    liatOrientation.moveSpecific(0.9);
                    specimenArm.scorePush();
                    follower.followPath(line10);
                    setPathState(12);
                }
                break;
            case 12:
                if (follower.getCurrentTValue() > 0.05) {
                    liatClaw.openClaw();
                }
                if (follower.getCurrentTValue() > 0.1) {
                    specimenArm.pickUp();
                }
                if (follower.getCurrentTValue() > 0.25) {
                    liatOrientation.movePick();
                }
                if (follower.getCurrentTValue() > 0.9) {
                    liatClaw.closeClaw();
                    follower.followPath(line12);
                    setPathState(13);
                }
                break;
            case 13:
                if (follower.getCurrentTValue() > 0.05) {
                    specimenArm.score();
                }
                if (follower.getCurrentTValue() > 0.3) {
                    liatOrientation.moveScore();
                }
                if (follower.getCurrentTValue() > 0.98 && follower.getTranslationalError().getMagnitude() < 2) {
                    liatOrientation.moveSpecific(0.9);
                    specimenArm.scorePush();
                    follower.followPath(line10);
                    setPathState(14);
                }
                break;
            case 14:
                if (follower.getCurrentTValue() > 0.05) {
                    liatClaw.openClaw();
                }
                if (follower.getCurrentTValue() > 0.1) {
                    specimenArm.pickUp();
                }
                if (follower.getCurrentTValue() > 0.25) {
                    liatOrientation.movePick();
                }
                if (follower.getCurrentTValue() > 0.9) {
                    liatClaw.closeClaw();
                    follower.followPath(line13);
                    setPathState(15);
                }
                break;
            case 15:
                if (follower.getCurrentTValue() > 0.05) {
                    specimenArm.score();
                }
                if (follower.getCurrentTValue() > 0.3) {
                    liatOrientation.moveScore();
                }
                if (follower.getCurrentTValue() > 0.98 && follower.getTranslationalError().getMagnitude() < 2) {
                    liatOrientation.moveSpecific(0.9);
                    specimenArm.scorePush();
                    follower.followPath(line10);
                    setPathState(16);
                }
                break;
            case 16:
                if (follower.getCurrentTValue() > 0.05) {
                    liatClaw.openClaw();
                }
                if (follower.getCurrentTValue() > 0.1) {
                    specimenArm.pickUp();
                }
                if (follower.getCurrentTValue() > 0.25) {
                    liatOrientation.movePick();
                }
                if (follower.getCurrentTValue() > 0.9) {
                    liatClaw.closeClaw();
                    follower.followPath(line14);
                    setPathState(17);
                }
                break;
            case 17:
                if (follower.getCurrentTValue() > 0.05) {
                    specimenArm.score();
                }
                if (follower.getCurrentTValue() > 0.3) {
                    liatOrientation.moveScore();
                }
                if (follower.getCurrentTValue() > 0.98 && follower.getTranslationalError().getMagnitude() < 2) {
                    specimenArm.scorePush();
                    waitTimer(500);
                    setPathState(18);
                }
                break;
            case 18:
                requestOpModeStop();
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        line2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(132, 114, Point.CARTESIAN),
                                new Point(110, 112, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(135))
                .build();

        line3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(110, 112, Point.CARTESIAN),
                                new Point(125.282, 114, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(45))
                .build();

        line4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(125.282, 114, Point.CARTESIAN),
                                new Point(110, 122, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(135))
                .build();

        line5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(110, 122, Point.CARTESIAN),
                                new Point(125.653, 122, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(45))
                .build();

        line6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(125.653, 122, Point.CARTESIAN),
                                new Point(107.5, 132, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(135))
                .build();

        line7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(107.5, 132, Point.CARTESIAN),
                                new Point(124.541, 130, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(45))
                .build();

        line8 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(124.541, 130, Point.CARTESIAN),
                                new Point(120, 122.5, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        line67 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(120, 122.5, Point.CARTESIAN),
                                new Point(129, 115, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        line9 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(129, 115, Point.CARTESIAN),
                                new Point(122.502, 70, Point.CARTESIAN),
                                new Point(100, 72.5, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        line10 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(100, 72.5, Point.CARTESIAN),
                                new Point(128, 117.5, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        line11 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(128, 117.5, Point.CARTESIAN),
                                new Point(122.502, 70, Point.CARTESIAN),
                                new Point(100, 72, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        line12 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(128, 117.5, Point.CARTESIAN),
                                new Point(122.502, 70, Point.CARTESIAN),
                                new Point(100, 72, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        line13 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(128, 117.5, Point.CARTESIAN),
                                new Point(122.502, 70, Point.CARTESIAN),
                                new Point(100, 72, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        line14 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(128, 117.5, Point.CARTESIAN),
                                new Point(122.502, 70, Point.CARTESIAN),
                                new Point(100, 72, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        // Get things from hardware map
        slides = new PIDFSlide(hardwareMap);
        //panning = new PIDFPanning(hardwareMap);
        claw = new Claw(hardwareMap);
        pitching = new Pitching(hardwareMap);
        orientation = new Orientation(hardwareMap);
        panningServo = new PanningServo(hardwareMap);
        sweeping = new Sweeping(hardwareMap);
        light = new Light(hardwareMap);
        panning = new PIDFPanning(hardwareMap);

        // Set up follower
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        buildPaths();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        waitTimer = new Timer();

        // Place parts in initial positions
        light.goToWhite();

        slides.setTargetPos(0);

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        specimenArm = new SpecimenArm2(hardwareMap);
        liatClaw = new LiatClaw(hardwareMap);
        liatOrientation = new LiatOrientation(hardwareMap);

        liatOrientation.moveSpecific(0.5);
        specimenArm.moveSpecific(0.15);
        liatClaw.closeClaw();

//        fl.setDirection(DcMotorSimple.Direction.REVERSE);
//        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        fr.setDirection(DcMotorSimple.Direction.FORWARD);
//        br.setDirection(DcMotorSimple.Direction.FORWARD);
        followerTime = new Timer();

        // Set path state to initial
        setPathState(0);

        while (opModeInInit()) {
            panning.setTargetPos(1000);
            slides.setPower(-0.2);
            panning.updatePanning();
            panning.update();
            telemetry.addData("X", xValue);
            telemetry.addData("Y", yValue);
            telemetry.addData("Orientation", sigmaOrientation);
            if (gamepad1.x) {
                xValue -= 0.25;
                sleep(200);
            }
            if (gamepad1.b) {
                xValue += 0.25;
                sleep(200);
            }
            if (gamepad1.y) {
                yValue += 0.25;
                sleep(200);
            }
            if (gamepad1.a) {
                yValue -= 0.25;
                sleep(200);
            }
            if (gamepad1.dpad_right) {
                sigmaOrientation += 0.1;
                sleep(200);
            }
            if (gamepad1.dpad_left) {
                sigmaOrientation -= 0.1;
                sleep(200);
            }
            telemetry.update();
        }

        waitForStart();
        limelight.pipelineSwitch(0);
        pitching.moveUp();
        orientation.moveSpecific(sigmaOrientation);
        panningServo.moveUp();
        sweeping.sweeperUp();
        xValue *= 1.1;
        if (yValue > 7) {
            yValue -= 1.5;
        } else if (yValue > 6) {
            yValue -= 0.75;
        } else if (yValue > 5) {
            yValue -= 0.5;
        } else if (yValue < 2.5) {
            yValue += 1;
        }

        if (xValue < 0.5) {
            pickUpPose = new Pose(110.5 - yValue, 74 + xValue, Math.toRadians(180));
        } else if (xValue > 0.5) {
            pickUpPose = new Pose(110.5 - yValue, 74.75  + xValue, Math.toRadians(180));
        }

        slideScore1 += (int) (yValue * 50);

        line1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(132.000, 80.000, Point.CARTESIAN),
                                new Point(96, 76.170, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
                .build();
        line420 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(96, 76.170, Point.CARTESIAN),
                                new Point(pickUpPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1.5)
                .setPathEndVelocityConstraint(0.01)
                .build();
        line69 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickUpPose),
                                new Point(132, 114, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                .build();

        sweeping.sweeperUp();
        follower.setPose(startPose);
        opmodeTimer.resetTimer();
        waitTimer.resetTimer();

        while (opModeIsActive()) {
            autonomousPathUpdate();
            follower.update();
            if (updateSigma) {
                //panning.updatePanning();
                slides.updateSlide();
                limelightStuff();
                updateLimelight();
            }
            if (sigmaSlide) {
                slides.setPower(-1);
            }

            // Feedback to Driver Hub
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.headingError);
            telemetry.addData("path: ", follower.getCurrentPath());
            telemetry.addData("closest point: ", follower.getClosestPose());
            telemetry.addData("translational: ", follower.getTranslationalError().getMagnitude());
            telemetry.addData("T value: ", follower.getCurrentTValue());
            //telemetry.addData("Panning value: ", panning.getCurrentPos());
            telemetry.addData("Slide target: ", slides.getTargetPos());
            telemetry.addData("Slide pos: ", slides.getCurrentPos());
            telemetry.update();
        }
    }

    public void updateImportant() {
        //panning.updatePanning();
        slides.updateSlide();
        follower.update();
    }

    public void limelightStuff() {
        if (limelight.isRunning()) {
            telemetry.addData("limelight is running", true);
        }
        result = limelight.getLatestResult();
        if (result == null) {
            telemetry.addData("Time: ", limelight.getTimeSinceLastUpdate());
            telemetry.addData("result", "null");
            limelight.close();
            limelight.start();
            limelight.pipelineSwitch(0);
        }
    }

    public void updateLimelight() {
        if (result != null) {
            updateResults();
        }
    }

    public void runTracking() {
        if (result != null) {
            followerTime.resetTimer();
            while (followerTime.getElapsedTime() < 750) {
                follower.update();
                updateResults();
                slides.setTargetPos(slides.getCurrentPos() + targetPosition);
                strafe(power);
                orientation.moveSpecific(uhorientation);
                slides.updateSlide();
            }
            strafe(0);
            panningServo.moveDown();
            waitTimer(250);
            pitching.moveDown();
            waitTimer(250);
            claw.closeClaw();
            waitTimer(250);
        }
    }

    public void strafe(double power) {
        fl.setPower(power * 0.8);
        br.setPower(power * 0.6);
        fr.setPower(-power * 0.8);
        bl.setPower(-power * 0.6);
    }

    public void runBackward(double power) {
        double newPower = power;
        fl.setPower(-newPower);
        br.setPower(-newPower);
        fr.setPower(-newPower);
        bl.setPower(-newPower);
    }

    public void updateResults() {
        result = limelight.getLatestResult();
        offsetX = result.getTx();
        offsetY = result.getTy();
        if (offsetX < -1 || offsetX > 1) {
            if (offsetX > 10 || offsetX < -10) {
                power = (offsetX + 4) / 30;
            } else {
                power = (offsetX + 4) / 45;
            }
        } else {
            power = 0;
        }
        if (offsetY == 0) {
            targetPosition = -100;
        } else {
            targetPosition = (int) ((offsetY) * 4);
        }

        pythonResults = result.getPythonOutput();
        uhorientation = pythonResults[6];

        telemetry.addData("offsetX: ", offsetX);
        telemetry.addData("offsetY: ", offsetY);
    }

    public void updateSystems() {
        //panning.updatePanning();
        slides.updateSlide();
        follower.update();
    }

    public void waitTimer(int timeMs) {
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() < timeMs) {updateSystems();}
    }
}
