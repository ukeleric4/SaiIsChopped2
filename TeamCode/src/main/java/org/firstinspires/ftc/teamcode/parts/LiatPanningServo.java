package org.firstinspires.ftc.teamcode.parts;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class LiatPanningServo {
    public ServoImplEx panning1; // Up - 0.9, Down - 0.4
    // Max - 0.138

    public LiatPanningServo(HardwareMap hw) {
        panning1 = hw.get(ServoImplEx.class, "rightPanning");
    }
    public void moveUp() {
        panning1.setPosition(0.75);
    }

    public void moveDown() {
        panning1.setPosition(0.13);
    }
    public double getPosition() {
        return panning1.getPosition();
    }

    public void moveSpecific(double pos) {
        panning1.setPosition(pos);
    }

    public void disable() {
        panning1.setPwmDisable();
    }

    public void enable() {
        panning1.setPwmEnable();
    }
}
