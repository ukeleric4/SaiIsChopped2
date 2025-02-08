package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.Light;
import org.firstinspires.ftc.teamcode.parts.Orientation;
import org.firstinspires.ftc.teamcode.parts.PIDFPanning;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.PanningServo;
import org.firstinspires.ftc.teamcode.parts.Pitching;

import com.pedropathing.util.Timer;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.vision.Vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="Red Teleop", group="Teleops")  //@Autonomous(...) is the other common choice
public class FinalTeleop extends LinearOpMode {
    private PIDFPanning panningMotor;
    private PIDFSlide slides;
    private Claw claw;
    private Orientation orientation;
    private PanningServo panningServo;
    private Pitching pitching;
    private Light light;
    private DistanceSensor dSensor;

    private double velocity = 0.6;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    PathChain idk;

    org.firstinspires.ftc.teamcode.vision.Vision vision;

    Timer pathTimer;
    Timer opModeTimer;
    boolean following = false;

    @Override
    public void runOpMode() throws InterruptedException {
        panningMotor = new PIDFPanning(hardwareMap);
        slides = new PIDFSlide(hardwareMap);
        claw = new Claw(hardwareMap);
        orientation = new Orientation(hardwareMap);
        panningServo = new PanningServo(hardwareMap);
        pitching = new Pitching(hardwareMap);
        dSensor = hardwareMap.get(DistanceSensor.class, "distance");
        light = new Light(hardwareMap);

        vision = new Vision(hardwareMap, telemetry, true, false, true);
        pathTimer = new Timer();
        opModeTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        waitForStart();
        // Move to normal positions
        while ((opModeInInit() || opModeIsActive()) && vision.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING);
        sleep(100);
        pathTimer.resetTimer();
        opModeTimer.resetTimer();
        light.goToBlue();
        pitching.moveUp();
        claw.openClaw();
        panningServo.moveDown();
        orientation.moveNormal();

        while (opModeIsActive()) {
            updateVelocity();
            pitching();
            hanging();
            manualPanning();
            clawMovement();
            specimenPosition();
            orientation();
            vision();
            slidePos();
            updateDistanceSensor();
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

    // Test function
    public void pitching() {
        if (gamepad1.dpad_down) {
            pitching.moveDown();
        } else if (gamepad1.dpad_up) {
            pitching.moveUp();
        }
    }

    public void hanging() {
        if (gamepad1.dpad_left && gamepad2.dpad_left) {
            slides.setTargetPos(600);
            while (slides.getCurrentPos() < 580) {
                slides.updateSlide();
                slides.updatePower();
            }
            panningMotor.setTargetPos(1800);
            while (panningMotor.getCurrentPos() < 1700) {
                panningMotor.updatePanning();
                slides.updateSlide();
                slides.updatePower();
            }
            slides.setTargetPos(1000);
            while (slides.getCurrentPos() < 980) {
                panningMotor.updatePanning();
                slides.updateSlide();
                slides.updatePower();
            }
            slides.setTargetPos(0);
            while (slides.getCurrentPos() > 600) {
                panningMotor.updatePanning();
                slides.updateSlide();
                slides.updatePower();
            }
            panningMotor.setTargetPos(500);
            while (slides.getCurrentPos() > 10) {
                panningMotor.updatePanning();
                slides.updateSlide();
                slides.updatePower();
            }
        }
    }

    public void slidePos() {
        if (gamepad2.y) {
            slides.setTargetPos(1500);
            panningServo.moveDown();
            claw.openClaw();
        } else if (gamepad1.left_bumper) {
            slides.setTargetPos(0);
        }
    }

    public void updateDistanceSensor() {
        double distance = dSensor.getDistance(DistanceUnit.CM);
        if (distance > 2.8 && slides.getCurrentPos() < 900 || slides.getCurrentPos() > 1100) {
            light.goToRed();
        } else if (distance < 2.8) {
            light.goToBlue();
        } else {
            light.goToRed();
        }
    }

    // Test function
    public void manualPanning() {
        if (gamepad2.right_trigger > 0.8) {
           panningMotor.setTargetPos(1600);
        } else if (gamepad2.left_trigger > 0.8) {
            panningMotor.setTargetPos(0);
        }
    }

    public void specimenPosition() {
        if (gamepad1.x) {
            // Open claw, then move to this position
            claw.openClaw();
            panningServo.moveUp();
            slides.setTargetPos(0);
            panningMotor.setTargetPos(1700);
        }
        if (gamepad1.b) {
            // Close claw first, then move to these positions
            claw.closeClaw();
            sleep(250);
            panningMotor.setTargetPos(450);
            slides.setTargetPos(1200);
        }
    }

    public void clawMovement() {
        if (gamepad2.dpad_down) {
            pitching.moveDown();
            sleep(400);
            claw.closeClaw();
            sleep(250);
            pitching.moveUp();
            sleep(500);
            if ((dSensor.getDistance(DistanceUnit.CM) < 2.8)) {
                slides.setTargetPos(0);
                while (slides.getCurrentPos() > 25) {
                    update();
                }
                panningMotor.setTargetPos(1600);
                while (panningMotor.getCurrentPos() < 1200) {update();}
                panningServo.moveSpecific(0.55);
                orientation.moveNormal();
                slides.setTargetPos(2300);
            } else {
                claw.openClaw();
            }
        } else if (gamepad2.dpad_up) {
            claw.openClaw();
            sleep(300);
            panningServo.moveDown();
            sleep(300);
            slides.setTargetPos(0);
            while (slides.getCurrentPos() > 1000) {
                update();
            }
            panningMotor.setTargetPos(0);
        }
    }

    public void vision() {
        if (gamepad1.a) {
            if (!following) {
                follower.setPose(new Pose(0,0, 0));
                follower.updatePose();

                double orientationPosition = vision.getOrientation();
                double yMovement = vision.getyMovement();
                double xMovement = vision.getxMovement();

                idk = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.getPose().getX(), follower.getPose().getY()), new Point(follower.getPose().getX() - yMovement, follower.getPose().getY() + xMovement)))
                        .setConstantHeadingInterpolation(0)
                        .setPathEndTranslationalConstraint(0.05)
                        .setPathEndTValueConstraint(0.999)
                        .setZeroPowerAccelerationMultiplier(2.5)
                        .build();

                orientation.moveSpecific(orientationPosition);
                follower.followPath(idk);
                pathTimer.resetTimer();
                following = true;
            }
        }
    }


    public void orientation() {
        if (gamepad2.b) {
            orientation.moveNormal();
        }
        if (gamepad2.x) {
            orientation.moveSideways();
        }
    }


    public void update() {
        if (opModeTimer.getElapsedTime() > 3000) {
            vision.updateVision();
        }

        if (following && pathTimer.getElapsedTime() > 500) {
            follower.startTeleopDrive();
            following = false;
        }

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * 0.4, true);

        panningMotor.updatePanning();
        slides.updateSlide();
        slides.updatePower();
        follower.update();
    }
}

