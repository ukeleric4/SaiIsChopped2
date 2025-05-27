package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.Light;
import org.firstinspires.ftc.teamcode.parts.Orientation;
import org.firstinspires.ftc.teamcode.parts.PIDFPanning;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.PanningServo;
import org.firstinspires.ftc.teamcode.parts.Pitching;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp
public class BucketTeleop extends LinearOpMode {
    private PIDFPanning panningMotor;
    private PIDFSlide slides;
    private Claw claw;
    private Orientation orientation;
    private PanningServo panningServo;
    private Pitching pitching;
    private Limelight3A limelight;
    //private Light light;

    private double velocity = 1.0;
    private double headingVelocity = 0.6;
    private boolean updatePanning;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    Timer opModeTimer;
    Timer waitTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        panningMotor = new PIDFPanning(hardwareMap);
        slides = new PIDFSlide(hardwareMap);
        claw = new Claw(hardwareMap);
        orientation = new Orientation(hardwareMap);
        panningServo = new PanningServo(hardwareMap);
        pitching = new Pitching(hardwareMap);
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //light = new Light(hardwareMap);

        opModeTimer = new Timer();
        waitTimer = new Timer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        waitForStart();
        opModeTimer.resetTimer();
        waitTimer.resetTimer();
        //light.goToBlue();
        pitching.moveUp();
        claw.openClaw();
        panningServo.moveDown();
        orientation.moveNormal();

        while (opModeIsActive()) {
            pickUp();
            slidePos();
            updateAll();
        }
    }

    public void slidePos() {
        if (gamepad2.y) {
            panningMotor.setTargetPos(0);
            pitching.moveUp();
            slides.setTargetPos(750);
            velocity = 0.25;
            headingVelocity = 0.25;
            panningServo.moveDown();
            claw.openClaw();
            waitTimerUpdate(500);
            updatePanning = false;
            panningMotor.setPower(0);
        }
    }

    public void waitTimerUpdate(int timeMs) {
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() < timeMs) {updateImportant();}
    }

    public void waitTimerFollowerUpdate(int timeMs) {
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() < timeMs) {updateFollower();}
    }

    public void pickUp() {
        if (gamepad2.a) {
            follower.breakFollowing();
            updatePanning = true;
            velocity = 1.0;
            headingVelocity = 0.6;
            pitching.moveDown();
            waitTimerFollowerUpdate(250);
            claw.closeClaw();
            waitTimerFollowerUpdate(150);
            panningServo.moveSpecific(0.6);
            pitching.moveUp();
            panningMotor.setTargetPos(0);
            waitTimerFollowerUpdate(100);
            follower.startTeleopDrive();
            if (claw.getEncoderPosition() > 160) {
                orientation.moveNormal();
                slides.setTargetPos(0);
            }
        }

        if (gamepad2.dpad_up) {
            updatePanning = true;
            claw.moveSpecific(0.8);
            panningMotor.setTargetPos(1900);
            while (panningMotor.getCurrentPos() < 1000) {
                updateAll();
            }
            orientation.moveSpecific(0.65);
            panningServo.moveSpecific(0.8);
            slides.setTargetPos(1360);
            velocity = 0.6;
            headingVelocity = 0.6;
        }

        if (gamepad2.dpad_down) {
            follower.breakFollowing();
            panningServo.moveSpecific(0.95);
            claw.openClaw();
            follower.breakFollowing();
            waitTimerUpdate(200);
            velocity = 1.0;
            panningServo.moveDown();
            follower.startTeleopDrive();
            waitTimerUpdate(250);
            slides.setTargetPos(0);
            while (slides.getCurrentPos() > 250) {
                updateAll();
            }
            panningMotor.setTargetPos(0);
        }

        if (gamepad2.left_stick_button) {
            follower.breakFollowing();
            claw.openClaw();
            waitTimerUpdate(200);
            velocity = 1.0;
            follower.startTeleopDrive();
            waitTimerUpdate(200);
            panningServo.moveDown();
            slides.setTargetPos(0);
            while (slides.getCurrentPos() > 250) {
                updateAll();
            }
            panningMotor.setTargetPos(0);
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

    public void updateLight() {
//        if (claw.getPosition() == 1.0) {
//            light.goToGreen();
//        }
//
//        if (claw.getPosition() == 0) {
//            light.goToBlue();
//        }
    }

    public void update() {
        if (updatePanning) {
            panningMotor.updatePanning();
        }
        panningMotor.update();
        if (panningMotor.getCurrentPos() < 0) {
            panningMotor.resetEncoder();
        }
        slides.updateSlide();
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * headingVelocity, true);
        follower.update();
        telemetry.addData("Slide pos: ", slides.getCurrentPos());
        telemetry.addData("Slide target: ", slides.getTargetPos());
        telemetry.addData("Panning pos: ", panningMotor.getCurrentPos());
        telemetry.addData("Panning target: ", panningMotor.getTargetPos());
    }

    public void updateAll() {
        orientation();
        update();
        telemetry.update();
    }

    public void updateImportant() {
        if (updatePanning) {
            panningMotor.updatePanning();
        }
        panningMotor.update();
        slides.updateSlide();
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * headingVelocity, true);
        follower.update();
        telemetry.update();
    }

    public void updateFollower() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * headingVelocity, true);
        follower.update();
    }
}
