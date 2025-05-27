package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpecimenArm {
    LiatPanningServo panning1;
    LiatPanningServo2 panning2;

    public SpecimenArm(HardwareMap hw) {
        panning1 = new LiatPanningServo(hw);
        panning2 = new LiatPanningServo2(hw);
    }

    public void pickUp() {
        panning2.moveDown();
    }

    public void score() {
        panning2.moveUp();
    }

    public void scorePush() {
        panning2.moveSpecific(1);
    }

    public void moveSpecific(double pos) {
        panning2.moveSpecific(pos);
    }
}
