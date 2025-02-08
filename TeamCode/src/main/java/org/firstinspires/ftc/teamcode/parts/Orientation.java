package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Orientation {
    public Servo orientation;

    public Orientation(HardwareMap hw) {
        orientation = hw.get(Servo.class, "orientation");
    }

    public void moveNormal() {
        orientation.setPosition(1.0);
    }

    public void moveSideways() {
        orientation.setPosition(0.5);
    }

    public void moveOpposite() {
        orientation.setPosition(0);
    }

    public void moveSpecific(double pos) {
        orientation.setPosition(pos);
    }
}
