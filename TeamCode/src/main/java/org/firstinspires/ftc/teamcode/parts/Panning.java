package org.firstinspires.ftc.teamcode.parts;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Timer;
import java.util.TimerTask;

public class Panning {
    public DcMotor panningMotor;

    public Panning(HardwareMap hardwareMap) {
        panningMotor = hardwareMap.get(DcMotor.class, "panningmotor");
    }

    public void runUp(int timeMs) {
        panningMotor.setPower(1.0);
        //time input
        if (timeMs > 0) {
            try {
                sleep(timeMs);
            } catch (InterruptedException e) {
            }
            stop();
        }
    }

    public void runDown(int timeMs) {
        panningMotor.setPower(-1.0);
        //time input
        if (timeMs > 0) {
            try {
                sleep(timeMs);
            } catch (InterruptedException e) {
            }
            stop();
        }
    }

    public void runForTime(Runnable action, int timeMs) {
        Timer timer = new Timer();
        TimerTask stopTask = new TimerTask() {
            @Override
            public void run() {
                // Stop the motor after the time
                panningMotor.setPower(0);
                timer.cancel();
            }
        };

        // Run the action (start the motor)
        action.run();
        timer.schedule(stopTask, timeMs);
    }

    public void runMotor(double power, int timeMs) {
        runForTime(() -> panningMotor.setPower(power), timeMs);
    }

    public void stop() {
        panningMotor.setPower(0);
    }
}
