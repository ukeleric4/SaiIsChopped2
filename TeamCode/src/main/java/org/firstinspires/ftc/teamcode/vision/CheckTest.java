package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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

import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.Light;
import org.firstinspires.ftc.teamcode.parts.Orientation;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.Panning;
import org.firstinspires.ftc.teamcode.parts.PanningServo;
import org.firstinspires.ftc.teamcode.parts.Pitching;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.util.SlideTest;

@TeleOp
public class CheckTest extends LinearOpMode {
    private Limelight3A limelight;
    private PIDFSlide slides;
    private Orientation orientation;
    private Pitching pitching;
    private PanningServo panning;
    private Claw claw;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private PathChain align;
    boolean following = false;

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
    int targetPosition = 1250;

    Light light;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        orientation = new Orientation(hardwareMap);
        pitching = new Pitching(hardwareMap);
        panning = new PanningServo(hardwareMap);
        orientation = new Orientation(hardwareMap);
        claw = new Claw(hardwareMap);
        slides = new PIDFSlide(hardwareMap);
        light = new Light(hardwareMap);

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        pitching.moveUp();
        light.goToWhite();
        claw.openClaw();
        panning.moveSpecific(0.5);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        followerTime = new Timer();

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();
        followerTime.resetTimer();

        while (opModeIsActive()) {
            result = limelight.getLatestResult();

            if (gamepad1.y) {
                slides.setTargetPos(1250);
            }

            if (gamepad1.x) {
                claw.openClaw();
                panning.moveSpecific(0.5);
            }

            updateLimelight();

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

            slides.updateSlide();
            slides.updatePower();
            follower.update();
            telemetry.addData("Current Position X: ", follower.getPose().getX());
            telemetry.addData("Current Position Y: ", follower.getPose().getY());
            telemetry.addData("Current Position heading: ", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    public void updateLimelight() {
        if (result != null) {
            updateResults();

            if (gamepad1.a) {
                followerTime.resetTimer();
                while (followerTime.getElapsedTime() < 1500) {
                    updateResults();
                    slides.setTargetPos(slides.getCurrentPos() + targetPosition);
                    strafe(power);
                    orientation.moveSpecific(uhorientation);
                    slides.updateSlide();
                    slides.updatePower();
                }
                strafe(0);
                    panning.moveDown();
                    sleep(300);
                    pitching.moveDown();
                    sleep(200);
                    claw.closeClaw();
                    pitching.moveUp();
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
        result = limelight.getLatestResult();
        offsetX = result.getTx();
        offsetY = result.getTy();
        if (offsetX < -1 || offsetX > 1) {
            if (offsetX > 10 || offsetX < -10) {
                power = (offsetX + 4) / 30;
            } else {
                power = (offsetX + 4) / 15;
            }
        } else {
            power = 0;
        }
        targetPosition = (int) ((-13 + offsetY) * 3);

        pythonResults = result.getPythonOutput();
        uhorientation = pythonResults[6];

        telemetry.addData("offsetX: ", offsetX);
        telemetry.addData("offsetY: ", offsetY);
    }
}
