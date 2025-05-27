package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.parts.HangingServos;

@TeleOp
public class HangTest extends LinearOpMode {
    private HangingServos hanging;
    DcMotor bl;
    DcMotor br;

    @Override
    public void runOpMode() throws InterruptedException {
        hanging = new HangingServos(hardwareMap);
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            hanging.moveDown();
            sleep(3000);
            bl.setPower(-1);
            br.setPower(-1);
        }
    }
}
