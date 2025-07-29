package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private Servo wristRight, wristLeft;

    public Arm(HardwareMap hardwareMap) {
        wristRight = hardwareMap.get(Servo.class, "wristRight");
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
    }

    // Custom Action to move the servos to a specific position with smooth movement
    public Action moveToPositionActionArm(double leftTarget, double rightTarget) {
        return new Action() {
            private boolean initialized = false;
            private double leftCurrent, rightCurrent;
            private static final double STEP = 0.01; // Adjust step size for slower/faster movement
            private static final long DELAY_MS = 20; // Delay between steps in milliseconds

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    // Get current positions of the servos
                    leftCurrent = wristLeft.getPosition();
                    rightCurrent = wristRight.getPosition();
                    initialized = true;
                }

                boolean leftDone = false, rightDone = false;

                // Gradually move the left servo
                if (Math.abs(leftCurrent - leftTarget) > STEP) {
                    leftCurrent += leftTarget > leftCurrent ? STEP : -STEP;
                    wristLeft.setPosition(leftCurrent);
                } else {
                    leftDone = true; // Mark as done when close enough
                }

                // Gradually move the right servo
                if (Math.abs(rightCurrent - rightTarget) > STEP) {
                    rightCurrent += rightTarget > rightCurrent ? STEP : -STEP;
                    wristRight.setPosition(rightCurrent);
                } else {
                    rightDone = true; // Mark as done when close enough
                }

                // Add telemetry for debugging
                packet.put("Wrist Left Current", leftCurrent);
                packet.put("Wrist Right Current", rightCurrent);
                packet.put("Wrist Left Target", leftTarget);
                packet.put("Wrist Right Target", rightTarget);

                // If both servos are done moving, the action is complete
                return !(leftDone && rightDone);
            }
        };
    }
}
