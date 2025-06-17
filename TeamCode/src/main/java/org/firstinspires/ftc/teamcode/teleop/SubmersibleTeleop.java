package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.HeadingPid;
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
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.*;

@TeleOp
public class SubmersibleTeleop extends LinearOpMode {
    //private Limelight3A limelight;
    private PIDFPanning thosewhoknow;
    private PIDFPanning panningMotor;
    private PIDFSlide slides;
    private Claw claw;
    private Orientation orientation;
    private PanningServo panningServo;
    private Pitching pitching;
    private SpecimenArm specimenArm;
    private LiatOrientation liatOrientation;
    private LiatClaw liatClaw;
    private HeadingPid headingPid;
    //private Light light;

    private double velocity = 1.0;
    private double headingVelocity = 0.4;
    private boolean updatePanning = false;
    private boolean robotCentric = true;

    private boolean headingLock = false;
    private double headingCorrection;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    Timer opModeTimer;
    Timer waitTimer;

    Timer followerTime;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    @Override
    public void runOpMode() throws InterruptedException {
        panningMotor = new PIDFPanning(hardwareMap);
        slides = new PIDFSlide(hardwareMap);
        claw = new Claw(hardwareMap);
        orientation = new Orientation(hardwareMap);
        panningServo = new PanningServo(hardwareMap);
        pitching = new Pitching(hardwareMap);
        specimenArm = new SpecimenArm(hardwareMap);
        liatClaw = new LiatClaw(hardwareMap);
        liatOrientation = new LiatOrientation(hardwareMap);

        opModeTimer = new Timer();
        waitTimer = new Timer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        headingPid = new HeadingPid();

        followerTime = new Timer();

        //specimenArm.panning1.disable();
        specimenArm.panning2.enable();

        waitForStart();
        //limelight.pipelineSwitch(0);
        followerTime.resetTimer();
        opModeTimer.resetTimer();
        waitTimer.resetTimer();
        pitching.moveUp();
        claw.openClaw();
        panningServo.moveUp();
        orientation.moveNormal();
        liatOrientation.movePick();
        specimenArm.pickUp();
        liatClaw.openClaw();

        while (opModeIsActive()) {
            pickUp();
            slidePos();
            updateAll();
        }
    }

    public void slidePos() {
        if (gamepad2.y) {
            if (slides.getCurrentPos() < 100 || slides.getTargetPos() == 750) {
                velocity = 0.25;
                headingVelocity = 0.25;
                panningMotor.setPower(0);
                updatePanning = false;
                robotCentric = true;
                panningMotor.setTargetPos(0);
                pitching.moveUp();
                slides.setTargetPos(750);
                claw.openClaw();
                if (panningServo.getPosition() == 0) {
                    panningServo.moveUp();
                } else {
                    panningServo.moveDown();
                }
                panningMotor.setPower(0);
                waitTimerUpdate(500);
            }
        }
    }

    public void hang() {
        if (gamepad1.right_stick_button && gamepad2.right_stick_button) {
            updatePanning = true;
            panningMotor.setTargetPos(1900);
            while (panningMotor.getCurrentPos() < 1000) {
                updateAll();
            }
            slides.setTargetPos(1250);
            specimenArm.scorePush();
            liatOrientation.movePick();
        }
        if (gamepad1.left_stick_button && gamepad2.left_stick_button) {
            slides.setTargetPos(0);
            waitTimerUpdate(1000);
            specimenArm.pickUp();
        }
    }

    public void waitTimerUpdate(int timeMs) {
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() < timeMs) {updateImportant();}
    }

    public void sigmaWaitUpdate(int timeMs) {
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() < timeMs) {updateAll();}
    }

    public void waitTimer(int timeMs) {
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() < timeMs) {}
    }

    public void waitTimerFollowerUpdate(int timeMs) {
        waitTimer.resetTimer();
        while (waitTimer.getElapsedTime() < timeMs) {updateFollower();}
    }

    public void pickUp() {
        if (gamepad2.a) {
            updatePanning = false;
            follower.breakFollowing();
            pitching.moveDown();
            waitTimerUpdate(100);
            claw.closeClaw();
            waitTimerUpdate(250);
            pitching.moveUp();
            panningMotor.setTargetPos(0);
            follower.startTeleopDrive();
            if (claw.getEncoderPosition() > 180) {
                velocity = 1.0;
                headingVelocity = 0.4;
                panningServo.moveSpecific(0.6);
                orientation.moveNormal();
                slides.setTargetPos(0);
            } else {
                claw.openClaw();
                orientation.moveNormal();
            }
        }

        if (gamepad2.dpad_up) {
            updatePanning = false;
            panningMotor.setPower(1);
            claw.moveSpecific(0.8);
            panningMotor.setTargetPos(1900);
            orientation.moveSpecific(0.5);
            panningServo.moveUp();
            headingVelocity = 0.4;
        }

        if (gamepad2.dpad_down) {
            claw.openClaw();
            waitTimerUpdate(500);
            panningMotor.setPower(-1);
            panningServo.moveUp();
            waitTimerUpdate(500);
            panningMotor.setPower(0);


        }

        if (gamepad2.left_trigger > 0.8 ) {
            slides.setTargetPos(0);
            velocity = 0.75;
        }
    }

    public void score() {
        if (gamepad2.left_bumper) {
            //specimenArm.panning1.disable();
            liatClaw.openClaw();
            specimenArm.scorePush();
            pitching.moveUp();
        }

        if (gamepad2.right_trigger > 0.8) {
            //specimenArm.panning1.disable();
            headingVelocity = 0.25;
            velocity = 0.75;
            liatOrientation.movePick();
           /*if (specimenArm.panning2.getPosition() == 1) {
               specimenArm.moveSpecific(0.55);
               waitTimerUpdate(400);
           } else if (specimenArm.panning1.getPosition() == 0.75) {
               specimenArm.moveSpecific(0.4);
               waitTimerUpdate(400);
           }*/
            specimenArm.moveSpecific(0.55);
            waitTimerUpdate(400);
            specimenArm.pickUp();
            liatClaw.openClaw();
        }

        if (gamepad2.right_bumper) {
            updatePanning = false;
            slides.setPower(0);
            panningMotor.setPower(0);
            velocity = 0.6;
            headingVelocity = 0.4;
            liatClaw.closeClaw();
            waitTimerUpdate(600);
            specimenArm.score();
            panningServo.moveUp();
            waitTimerUpdate(800);
            liatOrientation.moveScore();
        }
    }

    public void orientation() {
        if (gamepad2.b) {
            orientation.moveNormal();
        }
        if (gamepad2.x) {
            orientation.moveSideways();
        }

        if (gamepad1.left_trigger > 0.8) {
            velocity = 0.3;
        }
        if (gamepad1.right_trigger > 0.8) {
            velocity = 1;
        }

        if (gamepad1.a) {
            //specimenArm.panning1.disable();
        }
        if (gamepad1.y) {
            specimenArm.panning1.enable();
        }
    }

    public void checkHeadingLock() {
        if (headingLock) {
            double targetHeading = Math.toRadians(180);
            double currentHeading = follower.getPose().getHeading();
            double headingErr = targetHeading - currentHeading;
            headingErr = Math.IEEEremainder(headingErr, 2 * Math.PI);

            if (Math.abs(headingErr) < Math.toRadians(2)) {
                headingCorrection = 0;
            } else {
                headingCorrection = headingPid.calculate(headingErr, targetHeading);
            }

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, gamepad1.left_stick_x * velocity, headingCorrection, robotCentric);

        } else {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * headingVelocity, robotCentric);
        }
    }

    public void update() {
        panningMotor.update();
        if (updatePanning) {
            panningMotor.updatePanning();
        }
        slides.updateSlide();
        checkHeadingLock();
        follower.update();
        telemetry.addData("Slide pos: ", slides.getCurrentPos());
        telemetry.addData("Slide target: ", slides.getTargetPos());
        telemetry.addData("Panning pos: ", panningMotor.getCurrentPos());
        telemetry.addData("Panning target: ", panningMotor.getTargetPos());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
    }

    public void updateAll() {
        orientation();
        //updateLimelight();
        hang();
        score();
        update();
        telemetry.update();
    }

    public void updateImportant() {
        panningMotor.update();
        if (updatePanning) {
            panningMotor.updatePanning();
        }
        slides.updateSlide();
        checkHeadingLock();
        //follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * headingVelocity, !robotCentric);
        follower.update();
        telemetry.update();
    }


    public void updateFollower() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * headingVelocity, !robotCentric);
        follower.update();
    }
}
