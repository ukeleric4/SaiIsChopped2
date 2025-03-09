package org.firstinspires.ftc.teamcode.parts;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PIDFPanning {
    private PIDController controller;

    public static double p = 0.01, i = 0, d = 0.00001;
    public static double f = 0.005;

    public int target = 0;
    double ticks_in_degree = 1993.6 / 180.0;

    public double power = 0;
    public double offset = 0;

    private DcMotorEx motor1;

    public PIDFPanning(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);

        motor1 = hardwareMap.get(DcMotorEx.class, "panningmotor");
    }

    public void updatePanning() {
        controller.setPID(p, i, d);
        int motorPos = motor1.getCurrentPosition();
        double pid = controller.calculate(motorPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        power = pid + ff;
        motor1.setPower(power);
    }

    public void resetEncoder() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPower(double power) {
        motor1.setPower(power);
    }

    public void runDown() {
        motor1.setPower(-1);
    }

    public void runUp() {
        motor1.setPower(1);
    }

    public int getTargetPos() {
        return target;
    }

    public void setTargetPos(int targetPos) {
        target = (int) (targetPos + offset);
    }
    public void setOffset(double offset) {
        this.offset = offset;
    }
    public int getCurrentPos() {
        return motor1.getCurrentPosition();
    }
}

