package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.Light;
import org.firstinspires.ftc.teamcode.parts.Orientation;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.PanningServo;
import org.firstinspires.ftc.teamcode.parts.Pitching;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.List;
import java.util.Objects;

@TeleOp
public class LimeLightTest3 extends LinearOpMode {
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
    List<LLResultTypes.DetectorResult> detectorResults;
    List<List<Double>> corners;
    double[] pythonResults;
    double uhorientation;

    LLResult result;
    Light light;

    Timer followerTime;

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

        light.goToWhite();
        pitching.moveUp();
        claw.openClaw();
        panning.moveSpecific(0.5);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        followerTime = new Timer();

        limelight.pipelineSwitch(1);
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

            if (result != null) {
                    // Code goes here
                    detectorResults = result.getDetectorResults();
                    if (!detectorResults.isEmpty()) {
                        if (Objects.equals(detectorResults.get(0).getClassName(), "red")) {
                            offsetX = detectorResults.get(0).getTargetXDegrees();
                            if (offsetX > 0) {
                                offsetX *= 0.075;
                            } else {
                                offsetX *= 0.075;
                            }
                            offsetY = detectorResults.get(0).getTargetYDegrees();
                            offsetY = (-0.00098 * Math.pow(offsetY, 2) + 0.1744 * offsetY - 6.5) * 36;
                            corners = detectorResults.get(0).getTargetCorners();
                        } else {
                            offsetX = -1;
                            offsetY = -200;
                        }
                    }

                    telemetry.addData("offsetX: ", offsetX);
                    telemetry.addData("offsetY: ", offsetY);
                    if (corners != null && !corners.isEmpty()) {
                        double x1 = corners.get(0).get(0);
                        double x2 = corners.get(1).get(1);
                        double xDifference = x2 - x1;
                        if (xDifference < -80) {
                            uhorientation = 1.0;
                        } else {
                            uhorientation = 0.5;
                        }
                        telemetry.addData("x dif: ", xDifference);
                    }

                    if (gamepad1.a) {
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
                        }
                    }
                }

            if (following && followerTime.getElapsedTime() > 750) {
                pitching.moveDown();
                sleep(200);
                claw.closeClaw();
                pitching.moveUp();
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
}
