package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.parts.Orientation;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.Vision;

@TeleOp
public class VisionTest2 extends LinearOpMode {
    Vision vision;
    Orientation orientation;
    PIDFSlide slide;


    Gamepad oldGamepad1;
    Gamepad oldGamepad2;

    double newX;
    double newY;
    double newYPixels;
    double finalAddition;
    double multiplier = 2;

    @Override
    public void runOpMode() {
        vision = new Vision(hardwareMap, telemetry);
        slide = new PIDFSlide(hardwareMap);

        slide.setTargetPos(400);

        waitForStart();
        while (opModeIsActive()) {
            newX = vision.getXMovement();
            newY = vision.getYMovement();
            newYPixels = vision.getYPixels();

            if (gamepad1.y) {
                slide.setTargetPos(1000);
            }
            if (gamepad1.x && !oldGamepad1.a) {
                finalAddition = multiplier * newYPixels;
                if (newYPixels > 20) {
                    finalAddition -= 150;
                } else if (newYPixels < -20) {
                    finalAddition -= 150;
                }
                // newPosition.setX(currentPosition.getX() + newX);

                // run slide to position where block is
                slide.setTargetPos((int) (slide.getCurrentPos() - finalAddition));

                // Somehow make robot go to that position
            }

            vision.updateVision();
            oldGamepad1 = gamepad1;
            oldGamepad2 = gamepad2;
            telemetry.addData("current pos:", slide.getCurrentPos());
            telemetry.addData("Change in Y Pixels:", newYPixels);
            telemetry.addData("Y movement:", newY);
            telemetry.addData("New slide position:", slide.getCurrentPos() - newY);
            telemetry.update();
            slide.updateSlide();
            slide.updatePower();
        }
    }
}
