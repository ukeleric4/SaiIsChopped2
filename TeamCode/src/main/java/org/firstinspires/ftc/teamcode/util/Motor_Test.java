package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Motor_Test extends OpMode {
    DcMotor motor1;
    DcMotor motor2;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "slide1");
        motor2 = hardwareMap.get(DcMotor.class, "slide2");
    }

    @Override
    public void loop() {
        if (gamepad1.b) {
            motor1.setPower(1.0);
        } else if (gamepad1.x) {
            motor1.setPower(-1.0);
        }

        if (gamepad1.y) {
            motor2.setPower(1.0);
        } else if (gamepad1.a) {
            motor2.setPower(-1.0);
        }
    }
}
