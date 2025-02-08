package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.parts.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@TeleOp(name="Robot Alignment", group="Brubrhrubruh")
public class RobotAlignment extends LinearOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    Orientation orientation;
    PathChain idk;

    Vision vision;

    Timer pathTimer;
    Timer opModeTimer;
    boolean following = false;

    private double orientationPosition = 1.0;
    private double angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new Vision(hardwareMap, telemetry, true, false, false);
        pathTimer = new Timer();
        opModeTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        orientation = new Orientation(hardwareMap);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        waitForStart();
        while ((opModeInInit() || opModeIsActive()) && vision.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING);
        sleep(100);
        pathTimer.resetTimer();
        opModeTimer.resetTimer();

        while (opModeIsActive()) {
            if (opModeTimer.getElapsedTime() > 3000) {
                vision.updateVision();
            }
            if (gamepad1.a) {
                if (!following) {
                    follower.setPose(new Pose(0,0, 0));
                    follower.updatePose();

                    orientationPosition = vision.getOrientation();
                    angle = vision.getAngle();
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

            if (following && pathTimer.getElapsedTime() > 500) {
                follower.startTeleopDrive();
                following = false;
            }

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

//            telemetry.addData("follower current y: ", follower.getPose().getX());
//            telemetry.addData("follower current x:", follower.getPose().getY());
//            telemetry.addData("going to y: ", vision.getyMovement());
//            telemetry.addData("going to x: ", vision.getxMovement());
//            telemetry.addData("path timer time: ", pathTimer.getElapsedTime());
            telemetry.addData("servo position: ", orientationPosition);
            telemetry.addData("angle: ", angle);
            follower.update();
        }
    }
}