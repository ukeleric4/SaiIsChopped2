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
import org.firstinspires.ftc.teamcode.parts.LiatClaw;
import org.firstinspires.ftc.teamcode.parts.LiatOrientation;
import org.firstinspires.ftc.teamcode.parts.Light;
import org.firstinspires.ftc.teamcode.parts.Orientation;
import org.firstinspires.ftc.teamcode.parts.PIDFPanning;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.PanningServo;
import org.firstinspires.ftc.teamcode.parts.Pitching;
import org.firstinspires.ftc.teamcode.parts.SpecimenArm;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp
public class SubmersibleTeleop extends LinearOpMode {
    private Limelight3A limelight;
    private PIDFPanning panningMotor;
    private PIDFSlide slides;
    private Claw claw;
    private Orientation orientation;
    private PanningServo panningServo;
    private Pitching pitching;
    private SpecimenArm specimenArm;
    private LiatOrientation liatOrientation;
    private LiatClaw liatClaw;
    //private Light light;

    private double velocity = 1.0;
    private double headingVelocity = 0.4;
    private boolean updatePanning = true;
    private boolean robotCentric;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    Timer opModeTimer;
    Timer waitTimer;

    Timer followerTime;
    int targetPosition = 750;

    boolean following = false;

    double sigmaHeading;

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

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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

        followerTime = new Timer();

        waitForStart();
        limelight.start();
        followerTime.resetTimer();
        opModeTimer.resetTimer();
        waitTimer.resetTimer();
        pitching.moveUp();
        claw.openClaw();
        panningServo.moveDown();
        orientation.moveNormal();
        liatOrientation.moveScore();
        liatClaw.openClaw();

        while (opModeIsActive()) {
            pickUp();
            slidePos();
            updateAll();
        }
    }

    public void slidePos() {
        if (gamepad1.y) {
            velocity = 0.25;
            headingVelocity = 0.25;
            updatePanning = false;
            robotCentric = true;
            panningMotor.setTargetPos(0);
            pitching.moveUp();
            slides.setTargetPos(750);
            panningServo.moveSpecific(0.5);
            claw.openClaw();
            waitTimerUpdate(500);
            panningMotor.setPower(0);
        }

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
                velocity = 0.25;
                headingVelocity = 0.25;
                claw.openClaw();
                panningServo.moveDown();
                panningMotor.setPower(0);
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
        }
        if (gamepad1.left_stick_button && gamepad2.left_stick_button) {
            slides.setTargetPos(0);
            waitTimerUpdate(1000);
            panningMotor.setTargetPos(0);
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
            velocity = 1.0;
            headingVelocity = 0.4;
            pitching.moveDown();
            waitTimerUpdate(250);
            claw.closeClaw();
            waitTimerUpdate(150);
            pitching.moveUp();
            panningMotor.setTargetPos(0);
            waitTimerUpdate(100);
            follower.startTeleopDrive();
            if (claw.getEncoderPosition() > 160) {
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
            panningServo.moveSpecific(0.5);
            waitTimerUpdate(500);
            panningMotor.setPower(0);

        }
    }

    public void score() {
        if (gamepad2.left_bumper) {
            liatClaw.openClaw();
            specimenArm.scorePush();
        }

        if (gamepad2.right_trigger > 0.8) {
            liatOrientation.movePick();
            specimenArm.pickUp();
            liatClaw.openClaw();
        }

        if (gamepad2.right_bumper) {
            velocity = 1;
            headingVelocity = 0.4;
            liatClaw.closeClaw();
            waitTimerUpdate(500);
            slides.setPower(0);
            panningMotor.setPower(0);
            specimenArm.score();
            waitTimerUpdate(500);
            liatOrientation.moveScore();
            panningServo.moveSpecific(0.5);
            pitching.moveUp();
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
        panningMotor.update();
        if (updatePanning) {
            panningMotor.updatePanning();
        }
        slides.updateSlide();
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * headingVelocity, robotCentric);
        follower.update();
        telemetry.addData("Slide pos: ", slides.getCurrentPos());
        telemetry.addData("Slide target: ", slides.getTargetPos());
        telemetry.addData("Panning pos: ", panningMotor.getCurrentPos());
        telemetry.addData("Panning target: ", panningMotor.getTargetPos());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
    }

    public void updateAll() {
        orientation();
        updateLimelight();
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
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * headingVelocity, robotCentric);
        follower.update();
        telemetry.update();
    }

    public void updateFollower() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * headingVelocity, robotCentric);
        follower.update();
    }

    public void updateLimelight() {
        if (limelight.isRunning()) {
            telemetry.addData("limelight is running", true);
        }
        result = limelight.getLatestResult();
        if (result == null) {
            limelight.close();
            limelight.start();
            limelight.pipelineSwitch(0);
            telemetry.addData("Time: ", limelight.getTimeSinceLastUpdate());
            telemetry.addData("result", "null");
        }

        if (result != null) {
            updateResults();

            if (gamepad1.a) {
                follower.breakFollowing();
                runBackward(0.2);
                waitTimerUpdate(250);
                runBackward(0);
                followerTime.resetTimer();
                while (followerTime.getElapsedTime() < 1000) {
                    updateResults();
                    slides.setTargetPos(slides.getCurrentPos() + targetPosition);
                    strafe(power);
                    orientation.moveSpecific(uhorientation);
                    slides.updateSlide();
                }
                strafe(0);
                panningServo.moveDown();
                waitTimerUpdate(300);
                pitching.moveDown();
                waitTimerUpdate(200);
                claw.closeClaw();
                follower.startTeleopDrive();
                waitTimerUpdate(200);
                pitching.moveUp();
                if (claw.getEncoderPosition() > 160) {
                    panningServo.moveUp();
                    orientation.moveNormal();
                    slides.setTargetPos(0);
                    velocity = 1;
                    headingVelocity = 0.4;
                } else {
                    slides.setTargetPos(targetPosition);
                    panningServo.moveSpecific(0.5);
                    orientation.moveNormal();
                }
            }
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
                power = (offsetX + 4) / 25;
            } else {
                power = (offsetX + 4) / 40;
            }
        } else {
            power = 0;
        }
        if (offsetY == 0) {
            targetPosition = -100;
        } else {
            targetPosition = (int) ((offsetY) * 2);
        }

        pythonResults = result.getPythonOutput();
        uhorientation = pythonResults[6];

        telemetry.addData("offsetX: ", offsetX);
        telemetry.addData("offsetY: ", offsetY);
    }
}
