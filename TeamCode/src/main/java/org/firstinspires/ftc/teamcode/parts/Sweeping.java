package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sweeping {
    public Servo sweeping;

    public Sweeping(HardwareMap hw) {
        sweeping = hw.get(Servo.class, "sweeping");
    }

    public void sweeperDown() {
        sweeping.setPosition(0);
    }

    public void sweeperUp() {
        sweeping.setPosition(1.0);
    }

    public double getPosition() { return sweeping.getPosition(); }

    public void moveSpecific(double pos) {
        sweeping.setPosition(pos);
    }
}
