package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Motor_Test extends LinearOpMode {
    DcMotor motor1;
    DcMotor motor2;

    double power;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "m1");
        motor2 = hardwareMap.get(DcMotor.class, "m2");
        waitForStart();
        while (opModeIsActive()) {
            power = gamepad1.left_stick_x;
            if (power == 0) {
                motor1.setPower(0);
                motor2.setPower(0);
            } else if (power < 0) {
                motor2.setPower(power);
                motor1.setPower(0);
            } else if (power > 0) {
                motor2.setPower(power);
                motor1.setPower(0);
            }
        }
    }
}
