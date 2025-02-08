package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = -0.0029811403078306427;
        TwoWheelConstants.strafeTicksToInches = -0.0029811403078306427;
        TwoWheelConstants.forwardY = 6;
        TwoWheelConstants.strafeX = 3;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "br";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "bl";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
    }
}




