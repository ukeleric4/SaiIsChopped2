package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.parts.PanningServo;
import org.firstinspires.ftc.teamcode.parts.Vision;

@Config
@TeleOp
public class PIDFSlideLimelight extends LinearOpMode {
    private PIDController controller;
    private Limelight3A limelight;

    public static double p = 0.0125, i = 0, d = 0.000001;
    public static double f = 0.04;

    public static int target = 500;
    double ticks_in_degree = 384.5 / 180.0;

    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private double power;

    private PanningServo panningServo;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);

        motor1 = hardwareMap.get(DcMotorEx.class, "slide1");
        motor2 = hardwareMap.get(DcMotorEx.class, "slide2");

        motor2.setDirection(DcMotorEx.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        panningServo = new PanningServo(hardwareMap);
        waitForStart();

        limelight.start();
        limelight.pipelineSwitch(0);

        while (opModeIsActive()) {
                panningServo.moveUp();
                LLResult result = limelight.getLatestResult();
                controller.setPID(p, i, d);
                int motorPos = motor2.getCurrentPosition();
                double pid = controller.calculate(motorPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

                power = pid + ff;

                if (gamepad1.a) {
                    if (result != null) {
                        double y = result.getTy();
                        if (y != 0) {
                            if (y >= -20 && y <= 15) {
                                power = -((-1.0 / 35.0) * y + (3.0 / 7.0)); // Slope: -1/35, Intercept: 3/7
                            } else if (y > 15 && y <= 30) {
                                power = -((-1.0 / 15.0) * y + 1.0); // Slope: -1/15, Intercept: 1
                            }
                        } else {
                            power = 0.25;
                        }
                        telemetry.addData("power: ", power);
                    } else {
                        power = 0;
                    }
                }

                if (result != null) {
                    telemetry.addData("y result: ", result.getTy());
                }

                if (result == null) {
                    limelight.close();
                    limelight.start();
                    limelight.pipelineSwitch(0);
                }

                motor1.setPower(-power);
                motor2.setPower(-power);

                telemetry.addData("Target Position: ", target);
                telemetry.addData("Current Position: ", motor2.getCurrentPosition());
                telemetry.update();
        }
    }
}

