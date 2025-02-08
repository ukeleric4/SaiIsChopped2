package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.Orientation;
import org.firstinspires.ftc.teamcode.parts.PanningServo;
import org.firstinspires.ftc.teamcode.parts.Pitching;

@TeleOp
public class ServoTest extends LinearOpMode {
    private Claw claw;
    private Orientation orientation;
    private PanningServo panningServo;
    private Pitching pitching;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = new Claw(hardwareMap);
        orientation = new Orientation(hardwareMap);
        panningServo = new PanningServo(hardwareMap);
        pitching = new Pitching(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                panningServo.moveDown();
            }
            if (gamepad1.y) {
                panningServo.moveUp();
            }

            if (gamepad1.x) {
                orientation.moveOpposite();
            }
            if (gamepad1.b) {
                orientation.moveNormal();
            }

            if (gamepad1.dpad_left) {
                claw.closeClaw();
            }
            if (gamepad1.dpad_right) {
                claw.openClaw();
            }

            if (gamepad1.dpad_up) {
                pitching.moveUp();
            }
            if (gamepad1.dpad_down) {
                pitching.moveDown();
            }
        }
    }
}
