package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LiatClaw {
    public Servo claw;
    public AnalogInput encoder;

    public LiatClaw(HardwareMap hw) {
        claw = hw.get(Servo.class, "liatclaw");
    }

    public void openClaw() {
        claw.setPosition(0);
    }

    public void closeClaw() {
        claw.setPosition(1.0);
    }

    public double getPosition() { return claw.getPosition(); }

    public void moveSpecific(double pos) {
        claw.setPosition(pos);
    }
}
