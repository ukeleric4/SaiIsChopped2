package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.Light;
import org.firstinspires.ftc.teamcode.parts.Orientation;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.Panning;
import org.firstinspires.ftc.teamcode.parts.PanningServo;
import org.firstinspires.ftc.teamcode.parts.Pitching;
import org.firstinspires.ftc.teamcode.parts.Vision;

@TeleOp(name="Bucket Teleop", group="Linear Opmode")  //@Autonomous(...) is the other common choice
public class teleop extends LinearOpMode {
    public Panning panningMotor;
    public Claw claw;
    public Orientation orientation;
    public PanningServo panningServo;
    public Pitching pitching;
    public Light light;
    public PIDFSlide slides;
    public Vision vision;
    public DistanceSensor dSensor;

    // gamepad old for button
    Gamepad oldGamepad1;
    Gamepad oldGamepad2;

    // random
    private double velocity;

    // auto to bucket stuff

    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses PLAY)
        slides = new PIDFSlide(hardwareMap);
        panningMotor = new Panning(hardwareMap);
        claw = new Claw(hardwareMap);
        orientation = new Orientation(hardwareMap);
        panningServo = new PanningServo(hardwareMap);
        pitching = new Pitching(hardwareMap);
        dSensor = hardwareMap.get(DistanceSensor.class, "distance");
        light = new Light(hardwareMap);

        orientation.moveNormal();
        pitching.moveDown();
        waitForStart();

        while (opModeIsActive()) {
            // Speed control using triggers
            updateVelocity();
            // Pitching
            pitching();
            // Orientation
            orientation();
            // Hanging auto
            hanging();
            // Panning on gamepad 2
            manualPanning();
            // Ai orientation
            /* cvOrientation(); */
            // Manual panning servo
            manualPanServo();
            // Claw on game pad 2 + pitching does down + panning servo
            clawMovement();
            // Bring panning up and make slide to go bucket pos
            // Bring slide down and panning down
            slidePos();
            // Update distance sensor
            updateDistanceSensor();
            hang();
            // Update pedro pathing follower, cv, panning, slide
            update();

            oldGamepad1 = gamepad1;
            oldGamepad2 = gamepad2;

            telemetry.addData("current pos: ", slides.getCurrentPos());
            telemetry.addData("current target: ", slides.getTargetPos());
            telemetry.addData("Distance: ", dSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("slide encoder pos:", slides.getCurrentPos());
            telemetry.update();
        }
    }

    // Teleop functions
    public void updateVelocity() {
        if (gamepad1.right_trigger > 0.8) {
            velocity = 1;
        } else if (gamepad1.left_trigger > 0.8) {
            velocity = 0.15;
        } else {
            velocity = 0.6;
        }
    }

    // Test function
    public void pitching() {
        if (gamepad1.dpad_down) {
            pitching.moveDown();
        } else if (gamepad1.dpad_up) {
            pitching.moveUp();
        }
    }

    public void hanging() {
        /*if (gamepad2.dpad_left) {
            // Start hanging
            slides.setTargetPos(1500);
            while (slides.getCurrentPos() < 1450) {
                slides.updateSlide();
                slides.updatePower();
            }
            slides.setPower(-1);
            panningMotor.runUp(800);
            telemetry.addData("phase: ", "1");
            telemetry.update();
            while(slides.getCurrentPos() > 700) {slides.updateSlide();}
            panningMotor.runDown(800);
            telemetry.addData("phase: ", "2");
            telemetry.update();
            while (slides.getCurrentPos() > 400) {slides.updateSlide();}
            telemetry.addData("phase: ", "3");
            telemetry.update();
            panningMotor.stop();
            sleep(1000);
            slides.setPower(0);
            sleep(1000);
            slides.setTargetPos(1750);
            while (slides.getCurrentPos() < 1500) {
                slides.updateSlide();
                slides.updatePower();
            }
            panningServo.moveToMin();
            while (slides.getCurrentPos() < 1725) {
                slides.updateSlide();
                slides.updatePower();
            }
            panningMotor.runUp(800);
            sleep(1000);
            panningMotor.runDown(800);
            sleep(1000);
            slides.setPower(-1);
//            slides.setPower(0);
//            panningMotor.runDown();
//            slides.setTargetPos(1500);
//            while (slides.getCurrentPos() < 1450) {
//                slides.updateSlide();
//                slides.updatePower();
//            }
        }*/
    }

    // Test function
    public void manualPanServo() {
        if (gamepad2.y) {
            panningServo.moveUp();  //Move up
        } else if (gamepad2.a) {
            panningServo.moveDown();//Move down
        }
    }

    public void slidePos() {
        if (gamepad2.y) {
            slides.setTargetPos(1500);
            panningServo.moveDown();
        } else if (gamepad1.left_bumper) {
            slides.setTargetPos(0);
        } else if (gamepad2.dpad_left) {
            claw.openClaw();
            sleep(500);
            panningServo.moveDown();
            sleep(500);
            slides.setTargetPos(0);
        } else if (gamepad1.right_bumper) {
            slides.setTargetPos(2222);
            panningServo.moveSpecific(0.45);
        }
    }

    public void updateDistanceSensor() {
        double distance = dSensor.getDistance(DistanceUnit.CM);
        if (distance < 5.5 && distance > 2 && slides.getCurrentPos() > 1600) {
            light.goToBlue();
        } else if (distance > 5.5 && slides.getCurrentPos() < 900 || slides.getCurrentPos() > 1100) {
            light.goToRed();
        } else if (distance < 5.5 && distance > 2) {
            light.goToGreen();
        } else {
            light.goToRed();
        }
    }

    // Test function
    public void manualPanning() {
        if (gamepad2.right_trigger > 0.8) {
           panningMotor.runUp(0);
        } else if (gamepad2.left_trigger > 0.8) {
            panningMotor.runDown(0);
        } else {
            panningMotor.stop();
        }
    }


    public void clawMovement() {
        if (gamepad2.dpad_down) {
            pitching.moveDown();
            sleep(400);
            claw.closeClaw();
            sleep(250);
            pitching.moveUp();
            sleep(500);
            if ((dSensor.getDistance(DistanceUnit.CM) < 5.5) && (dSensor.getDistance(DistanceUnit.CM) > 2)) {
                panningServo.moveSpecific(0.3);
                slides.setTargetPos(0);
            } else {
                claw.openClaw();
            }
        } else if (gamepad2.dpad_up) {
            claw.openClaw();
        }
    }

    public void hang() {

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


        slides.updateSlide();
        slides.updatePower();
    }
}

