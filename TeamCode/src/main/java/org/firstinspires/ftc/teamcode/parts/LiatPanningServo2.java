package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LiatPanningServo2 {
    public Servo panning2;

    public LiatPanningServo2(HardwareMap hw) {
        panning2 = hw.get(Servo.class, "leftPanning");
    }
    public void moveUp() {
        panning2.setPosition(0.75);
    }

    public void moveDown() {
        panning2.setPosition(0.13);
    }

    public double getPosition() {
        return panning2.getPosition();
    }

    public void moveSpecific(double pos) {
        panning2.setPosition(pos);
    }
}
