package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BrakePad {
    public Servo breakpad;
    // Up: 0.45
    // Down: 0.125

    public BrakePad(HardwareMap hw) {
        breakpad = hw.get(Servo.class, "brakepad");
    }

    public void moveUp() {
        breakpad.setPosition(0.45);
    }

    public void moveDown() {
        breakpad.setPosition(0.125);
    }

    public double getPosition() { return breakpad.getPosition(); }

    public void moveSpecific(double pos) {
        breakpad.setPosition(pos);
    }
}
