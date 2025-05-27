package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangingServos {
    public CRServo hangingServo1;
    public CRServo hangingServo2;

    public HangingServos(HardwareMap hw) {
        hangingServo1 = hw.get(CRServo.class, "hanging1");
        hangingServo2 = hw.get(CRServo.class, "hanging2");
    }

    public void moveDown() {
        hangingServo1.setPower(1);
        hangingServo2.setPower(-1);
    }

    public void moveUp() {
        hangingServo1.setPower(-1);
        hangingServo2.setPower(1);
    }

    public void setCRPower(double power) {
        hangingServo1.setPower(power);
        hangingServo2.setPower(-power);
    }
}
