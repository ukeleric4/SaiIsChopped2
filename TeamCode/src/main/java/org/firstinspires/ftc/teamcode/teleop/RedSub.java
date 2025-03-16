package org.firstinspires.ftc.teamcode.teleop;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="TeleOp Red Submersible", group="Teleops")
public class RedSub extends LinearOpMode {
    private PIDFPanning panningMotor;
    private PIDFSlide slides;
    private Claw claw;
    private Orientation orientation;
    private PanningServo panningServo;
    private Pitching pitching;
    private Limelight3A limelight;
    private Light light;

    private double velocity = 0.6;
    private double headingVelocity = 0.4;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private final Pose hangPose = new Pose(132, 120, 0);
    PathChain autoPlace;
    PathChain autoPlaceBack;
    PathChain align;
    PathChain weirdPlace;

    Timer pathTimer1;
    Timer pathTimer2;
    Timer opModeTimer;
    boolean following = false;
    LLResult result;

    boolean autoPlacing = false;
    int autoPlaceNum = 0;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    double offsetX;
    double offsetY;
    double[] pythonResults;
    double uhorientation;
    double power = 0;

    boolean depositDown = true;

    int run = 0;

    Timer followerTime;
    Timer waitTimer;
    int targetPosition = 1250;

    @Override
    public void runOpMode() throws InterruptedException {
        panningMotor = new PIDFPanning(hardwareMap);
        slides = new PIDFSlide(hardwareMap);
        claw = new Claw(hardwareMap);
        orientation = new Orientation(hardwareMap);
        panningServo = new PanningServo(hardwareMap);
        pitching = new Pitching(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        light = new Light(hardwareMap);

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        pathTimer1 = new Timer();
        pathTimer2 = new Timer();
        opModeTimer = new Timer();
        followerTime = new Timer();
        waitTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
        limelight.pipelineSwitch(0);
        limelight.start();

        autoPlace = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(130.000, 116.006, Point.CARTESIAN),
                                new Point(127.347, 81.698, Point.CARTESIAN),
                                new Point(116.767, 91.102, Point.CARTESIAN),
                                new Point(112.5, 78.367, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        weirdPlace = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(130.000, 116.006, Point.CARTESIAN),
                                new Point(126.955, 85.029, Point.CARTESIAN),
                                new Point(114.416, 87.380, Point.CARTESIAN),
                                new Point(112.5, 81.5, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        autoPlaceBack = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(114.000, 80.000, Point.CARTESIAN),
                                new Point(121.861, 80.131, Point.CARTESIAN),
                                new Point(108.147, 105.600, Point.CARTESIAN),
                                new Point(130.000, 116.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        align = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(136.000, 136.000, Point.CARTESIAN),
                                new Point(130.000, 116, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        waitForStart();
        pathTimer1.resetTimer();
        pathTimer2 = new Timer();
        opModeTimer.resetTimer();
        followerTime.resetTimer();
        waitTimer.resetTimer();
        light.goToWhite();
        pitching.moveUp();
        claw.openClaw();
        panningServo.moveDown();
        orientation.moveOpposite();

        while (opModeIsActive()) {
            result = limelight.getLatestResult();
            updateVelocity();
            pitching();
            manualPanning();
            clawMovement();
            orientation();
            specimenPositions();
            hanging();
            slidePos();
            sampleControl();
            updateLimelight();
            update();
            telemetry.update();
        }
    }

    // Teleop functions
    public void updateVelocity() {
        if (gamepad1.right_trigger > 0.8) {
            velocity = 1;
        } else if (gamepad1.left_trigger > 0.8) {
            velocity = 0.15;
        } else {
            velocity = 0.6;
        }
    }

    public void pitching() {
        if (gamepad1.dpad_down) {
            pitching.moveDown();
        } else if (gamepad1.dpad_up) {
            pitching.moveUp();
        }
    }

    public void slidePos() {
        if (gamepad2.y) {
            slides.setTargetPos(1500);
            panningServo.moveDown();
            claw.openClaw();
        } else if (gamepad1.right_bumper) {
            slides.setTargetPos(1600);
            claw.closeClaw();
            panningServo.moveSpecific(0.45);
        } else if (gamepad2.a) {
            slides.setTargetPos(1200);
            claw.openClaw();
            panningServo.moveSpecific(0.45);
        }
    }

    public void sampleControl() {
        if (gamepad2.right_trigger > 0.8 && gamepad2.dpad_left && depositDown) {
            slides.setTargetPos(0);
            while (slides.getCurrentPos() > 100) {
                updateImportant();
            }
            panningMotor.setTargetPos(1800);
            while (panningMotor.getCurrentPos() < 1200) {
                updateImportant();
            }
            slides.setTargetPos(2300);
            panningServo.moveSpecific(0.6);
            depositDown = false;
        }

        if (gamepad2.right_trigger > 0.8 && gamepad2.dpad_left && !depositDown) {
            claw.openClaw();
            waitTimer(300);
            panningServo.moveDown();
            waitTimer(200);
            slides.setTargetPos(0);
            while (slides.getCurrentPos() > 100) {
                updateImportant();
            }
            panningMotor.setTargetPos(0);
            while (panningMotor.getCurrentPos() > 200) {
                updateImportant();
            }
            depositDown = true;
        }
    }

    public void specimenPositions() {
        if (gamepad2.left_stick_button) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            slides.setTargetPos(0);
            pitching.moveUp();
            while (slides.getCurrentPos() > 350) {
                update();
            }
            claw.openClaw();
            orientation.moveNormal();
            panningMotor.setTargetPos(0);
            panningServo.moveSpecific(0.55);
        }

        if (gamepad2.right_stick_button && !gamepad2.right_bumper) {
            claw.closeClaw();
            waitTimer(250);
            follower.setPose(hangPose);
            autoPlacing = true;
            run += 1;
            while (!gamepad1.b) {
                runAutoPlacement();
                updateImportant();
            }
            autoPlaceNum = 0;
            follower.breakFollowing();
            follower.startTeleopDrive();
        }

        if (gamepad2.right_bumper && !gamepad2.right_stick_button) {
            claw.closeClaw();
            waitTimer(250);
            orientation.moveOpposite();
            panningServo.moveSpecific(0.9);
            panningMotor.setTargetPos(1800);
            slides.setTargetPos(850);
        }

        if (gamepad1.x) {
            follower.setPose(new Pose(136, 136, 0));
            follower.followPath(align, true);
        } else if (gamepad1.left_bumper){
            pitching.moveDown();
            slides.setTargetPos(0);
            panningServo.moveDown();
        }
    }

    public void runAutoPlacement() {
        switch (autoPlaceNum) {
            case 0:
                follower.followPath(autoPlace);
                orientation.moveOpposite();
                panningServo.moveSpecific(0.9);
                panningMotor.setTargetPos(1800);
                while (panningMotor.getCurrentPos() < 1300) {
                    updateImportant();
                }
                slides.setTargetPos(900);
                setPathState(1);
                break;
            case 1:
                if (follower.getCurrentTValue() > 0.9) {
                    slides.setTargetPos(0);
                    while (slides.getCurrentPos() > 425) {
                        updateImportant();
                    }
                    follower.followPath(autoPlaceBack);
                    claw.openClaw();
                    orientation.moveNormal();
                    panningMotor.setTargetPos(0);
                    while (panningMotor.getCurrentPos() > 500) {
                        updateImportant();
                    }
                    slides.setTargetPos(200 + (run * 32));
                    panningServo.moveSpecific(0.55);
                    setPathState(2);
                }
                break;
            case 2:
                if (follower.getCurrentTValue() > 0.88) {
                    if (run < 7) {
                        follower.breakFollowing();
                        waitTimer(350);
                        claw.closeClaw();
                        waitTimer(100);
                        follower.followPath(autoPlace);
                        orientation.moveOpposite();
                        panningServo.moveSpecific(0.9);
                        panningMotor.setTargetPos(1800);
                        while (panningMotor.getCurrentPos() < 1300) {
                            updateImportant();
                        }
                        slides.setTargetPos(875);
                        run += 1;
                    } else {
                        follower.breakFollowing();
                        waitTimer(350);
                        claw.closeClaw();
                        waitTimer(100);
                        follower.followPath(weirdPlace);
                        orientation.moveOpposite();
                        panningServo.moveSpecific(0.9);
                        panningMotor.setTargetPos(1800);
                        while (panningMotor.getCurrentPos() < 1300) {
                            updateImportant();
                        }
                        slides.setTargetPos(900);
                        run += 1;
                    }
                    setPathState(1);
                }
        }
    }

    public void setPathState(int pState) {
        autoPlaceNum = pState;
        pathTimer2.resetTimer();
    }

    public void manualPanning() {
//        if (gamepad2.right_trigger > 0.8) {
//            panningMotor.setTargetPos(1600);
//        } else if (gamepad2.left_trigger > 0.8) {
//            panningMotor.setTargetPos(0);
//        }
    }

    public void clawMovement() {
        if (gamepad2.dpad_down) {
            follower.breakFollowing();
            pitching.moveDown();
            waitTimer(400);
            claw.closeClaw();
            waitTimer(250);
            pitching.moveUp();
            waitTimer(500);
            follower.startTeleopDrive();
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * 0.4, true);
            orientation.moveOpposite();
            panningServo.moveSpecific(0.45);
            waitTimer(200);
            slides.setTargetPos(0);
        }

        if (gamepad2.dpad_up && !gamepad2.right_bumper) {
            claw.openClaw();
            waitTimer(300);
            panningServo.moveDown();
            slides.setTargetPos(0);
            while (slides.getCurrentPos() > 1000) {
                update();
            }
            panningMotor.setTargetPos(0);
        }
    }

    public void updateLimelight() {
        if (result != null) {
            updateResults();
            if (gamepad1.a) {
                panningServo.moveSpecific(0.5);
                limelight.pipelineSwitch(0);
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
    }

    public void strafe(double power) {
        fl.setPower(power);
        br.setPower(power);
        fr.setPower(-power);
        bl.setPower(-power);
    }

    public void updateResults() {
        limelight.pipelineSwitch(0);
        result = limelight.getLatestResult();
        offsetX = result.getTx();
        offsetY = result.getTy();
        if (offsetX < -1 || offsetX > 1) {
            if (offsetX > 6 || offsetX < -6) {
                power = (offsetX) / 40;
            } else {
                power = (offsetX) / 22.5;
            }
        } else {
            power = 0;
        }
        targetPosition = (int) ((-5 + offsetY) * 5);

        pythonResults = result.getPythonOutput();
        uhorientation = pythonResults[6];

        telemetry.addData("offsetX: ", offsetX);
        telemetry.addData("offsetY: ", offsetY);
        telemetry.addData("pos target: ", targetPosition);
        telemetry.addData("power: ", power);
        telemetry.addData("path state: ", autoPlaceNum);
    }

    public void waitTimer(int timeMs) {
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() < timeMs) {}
    }

    public void orientation() {
        if (gamepad2.b) {
            orientation.moveOpposite();
        }
        if (gamepad2.x) {
            orientation.moveSideways();
        }
    }

    public void hanging() {
        if (gamepad2.right_trigger > 0.8 && gamepad2.left_trigger > 0.8) {
            slides.setTargetPos(2350);
            panningMotor.setTargetPos(1800);
            while (!gamepad2.left_bumper && !gamepad2.right_bumper) {update();}
            slides.setTargetPos(1400);
            panningMotor.setTargetPos(0);
        }
    }

    public void updateImportant() {
        panningMotor.updatePanning();
        slides.updateSlide();
        slides.updatePower();
        follower.update();
        telemetry.addData("path state: ", autoPlaceNum);
        telemetry.update();
    }

    public void update() {
        if (claw.getPosition() == 1.0) {
            headingVelocity = 1.0;
            light.goToGreen();
        } else {
            headingVelocity = 0.8;
            light.goToWhite();
        }

        if (following && pathTimer1.getElapsedTime() > 500) {
            follower.startTeleopDrive();
            following = false;
        }

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * headingVelocity, true);

        if (panningMotor.getTargetPos() == 0 && panningMotor.getCurrentPos() < 100) {
            panningMotor.setPower(0);
        } else {
            panningMotor.updatePanning();
        }

        if (slides.getTargetPos() == 0 && slides.getCurrentPos() < 100) {
            slides.setPower(0);
        } else {
            slides.updateSlide();
            slides.updatePower();
        }

        follower.update();
    }
}
