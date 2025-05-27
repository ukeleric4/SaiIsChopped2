package org.firstinspires.ftc.teamcode.parts;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PIDFSlide {
    private PIDController controller;

    public static double p = 0.0075, i = 0, d = 0.00000001;
    public static double f = 0.04;

    public int target = 0;
    double ticks_in_degree = 145.1 / 180.0;

    public double power = 0;

    public DcMotorEx motor1;
    public DcMotorEx motor2;

    public PIDFSlide(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);

        motor1 = hardwareMap.get(DcMotorEx.class, "slide1");
        motor2 = hardwareMap.get(DcMotorEx.class, "slide2");

        // Max - 1410

        motor2.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void updateSlide() {
        controller.setPID(p, i, d);
        int motorPos = -motor2.getCurrentPosition();
        double pid = controller.calculate(motorPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        power = pid + ff;
        motor1.setPower(-power);
        motor2.setPower(-power);
    }

    public void updatePower() {
        motor1.setPower(-power);
        motor2.setPower(-power);
    }

    public void setPower(double power) {
        motor1.setPower(-power);
        motor2.setPower(-power);
    }

    public void resetEncoder() {
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runDown() {
        motor2.setPower(1);
        motor1.setPower(1);
    }

    public void runUp() {
        motor2.setPower(-1);
        motor1.setPower(-1);
    }

    public void setTargetPos(int targetPos) {
        target = targetPos;
    }

    public int getTargetPos() {
        return target;
    }

    public int getCurrentPos() {
        return -motor2.getCurrentPosition();
    }

    public void runForward() {
        target += 50;
    }

    public void runBackward() {
        if (target >= 70) {
            target -= 50;
        }
    }
}

