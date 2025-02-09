package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
    public ColorSensor dSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        dSensor = hardwareMap.get(ColorSensor.class, "distance");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("blue: ", dSensor.blue());
            telemetry.update();
        }
    }
}
