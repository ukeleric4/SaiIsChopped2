package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class LiatPanningServo2 {
    public ServoImplEx panning2;
    public ServoControllerEx controller;

    public LiatPanningServo2(HardwareMap hw) {
        panning2 = hw.get(ServoImplEx.class, "leftPanning");
    }
    public void moveUp() {
        panning2.setPosition(0.775);
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

    public void disable() {
        panning2.setPwmDisable();
    }

    public void enable() {
        panning2.setPwmEnable();
    }
}
