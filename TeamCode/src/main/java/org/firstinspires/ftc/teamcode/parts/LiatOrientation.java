package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LiatOrientation {
    public Servo orientation;

    public LiatOrientation(HardwareMap hw) {
        orientation = hw.get(Servo.class, "liatorientation");
    }

    public void moveScore() {
        orientation.setPosition(1.0);
    }

    public void movePick() {
        orientation.setPosition(0);
    }

    public void moveSpecific(double pos) {
        orientation.setPosition(pos);
    }
}
