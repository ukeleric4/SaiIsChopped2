package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceSensorTest extends LinearOpMode {
    public DistanceSensor dSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        dSensor = hardwareMap.get(DistanceSensor.class, "distance");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("distance: ", dSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
