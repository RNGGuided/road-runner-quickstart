package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;

    public Claw(HardwareMap hardwareMap) {
        // Initialize the claw servo
        claw = hardwareMap.get(Servo.class, "claw");
    }

    // Custom Action to set the claw's position
    public Action setPositionActionClaw(double position) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    // Set the claw's position
                    claw.setPosition(position);

                    // Add telemetry for debugging
                    packet.put("Claw Position", position);

                    initialized = true; // Mark as initialized
                }

                // This action is instantaneous; return false to signal it's complete
                return false;
            }
        };
    }
}
