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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.Orientation;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.Pitching;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp
public class LimelightTest2 extends OpMode {
    private Limelight3A limelight;
    private PIDFSlide slides;
    private Orientation orientation;
    private Pitching pitching;

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

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        orientation = new Orientation(hardwareMap);
        pitching = new Pitching(hardwareMap);
        orientation = new Orientation(hardwareMap);
        slides = new PIDFSlide(hardwareMap);

        pitching.moveUp();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        followerTime = new Timer();

        limelight.pipelineSwitch(0);
        limelight.close();
        limelight.start();
        limelight.reloadPipeline();

    }

    @Override
    public void loop() {
        result = limelight.getLatestResult();

        if (gamepad1.y) {
            slides.setTargetPos(1250);
        }

        telemetry.addData("Limelight Connected", limelight != null);
        telemetry.addData("Result Null", result == null);
        if (result != null) {
            telemetry.addData("Result Valid", result.isValid());
            telemetry.addData("Pipeline Index", result.getPipelineIndex());
        }

        if (result != null) {
            // Code goes here
            offsetX = result.getTx() * 0.1;
            offsetY = result.getTy() * 20;
            pythonResults = result.getPythonOutput();
            uhorientation = pythonResults[6];

            telemetry.addData("offsetX: ", offsetX);
            telemetry.addData("offsetY: ", offsetY);
            telemetry.addData("orientation: ", orientation);
            telemetry.addData("latency: ", result.getCaptureLatency());
            telemetry.addData("uhh", result.getPipelineType());
            telemetry.addData("valid", result.isValid());

            if (gamepad1.a) {
                if (!following) {
                    // Horizontal movement
                    follower.setCurrentPoseWithOffset(new Pose(0, 0,0));
                    follower.updatePose();
                    align = follower.pathBuilder()
                            .addPath(new BezierLine(new Point(0, 0), new Point(0, -offsetX)))
                            .setConstantHeadingInterpolation(0)
                            .setPathEndTranslationalConstraint(0.05)
                            .setPathEndTValueConstraint(0.999)
                            .setZeroPowerAccelerationMultiplier(2.5)
                            .build();

                    follower.followPath(align);
                    followerTime.resetTimer();
                    following = true;

                    // Vertical movement
                    slides.setTargetPos((int) (slides.getCurrentPos() + offsetY));
                }
            }
        }

        if (following && followerTime.getElapsedTime() > 1500) {
            follower.resetOffset();
            follower.startTeleopDrive();
            following = false;
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
