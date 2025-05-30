package org.firstinspires.ftc.teamcode.parts;

import com.arcrobotics.ftclib.controller.PIDController;

public class HeadingPid {
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.001;
    public static double f = 0.005;

    double ticks_in_degree = 384.5 / 180.0;

    public HeadingPid() {
        controller = new PIDController(p, i, d);
    }

    public double calculate(double error, double target) {
        controller.setPID(p, i, d);
        double pid = controller.calculate(error, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        return (pid + ff);
    }
}
