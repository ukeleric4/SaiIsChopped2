package org.firstinspires.ftc.teamcode.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class ServoTest extends LinearOpMode {
    private Orientation orientation;
    private PanningServo panningServo;
    private Pitching pitching;
    private Sweeping sweeping;
    private Claw claw;
    private LiatClaw liatClaw;
    private LiatPanningServo liatPanningServo;
    private LiatPanningServo2 liatPanningServo2;
    private LiatOrientation liatOrientation;
    private BrakePad breakPad;

    private HangingServos hanging;

    public static double panningPos = 0;
    public static double orientationPos = 0;
    public static double pitchingPos = 0;
    public static double sweepingPos = 0;
    public static double clawPos = 0;
    public static double breakPadPos = 0;
    public static double hangingPower = 0;
    public static double liatClawPos = 0;
    public static double liatPanningPos = 0;
    public static double liatPanningPos2 = 0;
    public static double liatOrientationPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = new Claw(hardwareMap);
        orientation = new Orientation(hardwareMap);
        panningServo = new PanningServo(hardwareMap);
        pitching = new Pitching(hardwareMap);
        sweeping = new Sweeping(hardwareMap);
        //breakPad = new BrakePad(hardwareMap);
        hanging = new HangingServos(hardwareMap);
        liatClaw = new LiatClaw(hardwareMap);
        liatOrientation = new LiatOrientation(hardwareMap);
        liatPanningServo = new LiatPanningServo(hardwareMap);
        liatPanningServo2 = new LiatPanningServo2(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            panningServo.moveSpecific(panningPos);
            orientation.moveSpecific(orientationPos);
            pitching.moveSpecific(pitchingPos);
            sweeping.moveSpecific(sweepingPos);
            claw.moveSpecific(clawPos);
//            breakPad.moveSpecific(breakPadPos);
            liatClaw.moveSpecific(liatClawPos);
            //liatPanningServo.moveSpecific(liatPanningPos);
            liatPanningServo2.moveSpecific(liatPanningPos2);
            liatOrientation.moveSpecific(liatOrientationPos);
            //hanging.setCRPower(hangingPower);

            telemetry.addData("claw encoder", claw.getEncoderPosition());
            telemetry.addData("hitec position: ", liatPanningServo.getPosition());
            telemetry.update();
        }
    }
}
