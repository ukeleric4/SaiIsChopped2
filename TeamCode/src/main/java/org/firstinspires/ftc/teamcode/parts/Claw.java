package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public Servo claw;

    public Claw(HardwareMap hw) {
        claw = hw.get(Servo.class, "claw");
    }

    public void openClaw() {
        claw.setPosition(0);
    }

    public void closeClaw() {
        claw.setPosition(1.0);
    }

    public double getPosition() { return claw.getPosition(); }

    public void moveClawToSpecificPos(double pos) {
        claw.setPosition(pos);
    }
}
