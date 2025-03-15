package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PanningServo {
    public Servo panning;

    public PanningServo(HardwareMap hw) {
        panning = hw.get(Servo.class, "panning");
    }

    public void moveUp() {
        panning.setPosition(1.0);
    }

    public void moveDown() {
        panning.setPosition(0);
    }

    public double getPosition() { return panning.getPosition(); }

    public void moveSpecific(double pos) {
        panning.setPosition(pos);
    }
}
