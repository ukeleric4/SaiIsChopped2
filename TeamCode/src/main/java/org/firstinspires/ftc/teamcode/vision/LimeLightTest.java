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
public class LimeLightTest extends LinearOpMode {
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

    double offsetX;
    double offsetY;
    double[] pythonResults;
    double uhorientation;

    LLResult result;

    Timer followerTime;

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

        pitching.moveUp();
        light.goToWhite();
        claw.openClaw();
        panning.moveSpecific(0.5);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
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
            }

            updateLimelight();

            if (following && followerTime.getElapsedTime() > 1250) {
                pitching.moveDown();
                sleep(200);
                claw.closeClaw();
                pitching.moveUp();
                sleep(300);
                orientation.moveOpposite();
                limelight.pipelineSwitch(2);
                sleep(500);
                result = limelight.getLatestResult();
                boolean picked = checkIfPickedUp();
                follower.resetOffset();
                follower.startTeleopDrive();
                following = false;
                if (picked) {
                    panning.moveSpecific(0.5);
                } else {
                    slides.setTargetPos(1250);
                    while (slides.getCurrentPos() < 1245) {
                        slides.updateSlide();
                        slides.updatePower();
                    }
                    slides.setPower(0);
                    panning.moveSpecific(0.5);
                    claw.openClaw();
                    sleep(300);
                    limelight.pipelineSwitch(0);
                    sleep(300);
                    result = limelight.getLatestResult();
                    updateLimelight();
                    if (result != null && offsetY != -110 && offsetY != -90) {
                        runPaths();
                    }
                }
            }

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

    public void runPaths() {
        if (!following) {
            // Horizontal movement
            follower.setPose(new Pose(0, 0, 0));
            follower.updatePose();
            align = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(0, 0), new Point(0, -offsetX)))
                    .setConstantHeadingInterpolation(0)
                    .setPathEndTranslationalConstraint(0.05)
                    .setPathEndTValueConstraint(0.999)
                    .setZeroPowerAccelerationMultiplier(2)
                    .build();

            follower.followPath(align);
            followerTime.resetTimer();
            following = true;

            // Vertical movement
            slides.setTargetPos((int) (slides.getCurrentPos() + offsetY));

            // Orientation
            orientation.moveSpecific(uhorientation);

            panning.moveDown();
            sleep(300);
        }
    }

    public void updateLimelight() {
        if (result != null) {
            // Code goes here
            offsetX = result.getTx() * 0.1;
            offsetY = result.getTy();
            offsetY = (-0.00098 * Math.pow(offsetY, 2) + 0.1744 * offsetY - 4.5) * 20;

            pythonResults = result.getPythonOutput();
            uhorientation = pythonResults[6];

            if (uhorientation > 0.25 && uhorientation < 0.75) {
                offsetY -= 70;
            }

            telemetry.addData("offsetX: ", offsetX);
            telemetry.addData("offsetY: ", offsetY);

            if (gamepad1.a) {
                claw.openClaw();
                runPaths();
            }
        }
    }

    public boolean checkIfPickedUp() {
        if (result != null && result.isValid()) {
            return true;
        } else {
            return false;
        }
    }
}
