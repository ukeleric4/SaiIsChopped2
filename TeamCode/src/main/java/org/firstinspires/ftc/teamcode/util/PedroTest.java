//package org.firstinspires.ftc.teamcode.util;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//@TeleOp
//public class PedroTest extends LinearOpMode {
//    Gamepad oldGamepad1;
//    Gamepad oldGamepad2;
//
//    boolean following = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        waitForStart();
//
//        while (opModeIsActive()) {
//            if (gamepad1.a) {
//                follower.breakFollowing();
//                goToBucket();
//                following = true;
//            }
//
//            follower.update();
//            oldGamepad1 = gamepad1;
//            oldGamepad2 = gamepad2;
//
//            telemetry.addData("following: ", following);
//            telemetry.update();
//        }
//
//    }
//    public void goToBucket() {
//        bucketPath = follower.pathBuilder()
//                .addPath(
//                        // Line 1
//                        new BezierCurve(
//                                new Point(72.906, 117.013, Point.CARTESIAN),
//                                new Point(67.267, 125.270, Point.CARTESIAN),
//                                new Point(18.931, 120.034, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
//        follower.followPath(bucketPath);
//    }
//}
