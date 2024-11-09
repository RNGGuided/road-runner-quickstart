package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    private DcMotorEx slideLeft, slideRight;

    private static final double MIN_HEIGHT = 16.0; // Minimum height in inches
    private static final double MAX_HEIGHT = 51.5; // Maximum height in inches
    private static final int MIN_TICKS = 0; // Encoder ticks at minimum height
    private static final int MAX_TICKS = 4400; // Encoder ticks at maximum height

    public Slides(HardwareMap hardwareMap) {
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        slideLeft.setDirection(DcMotor.Direction.FORWARD);
        slideRight.setDirection(DcMotor.Direction.REVERSE);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Convert inches to encoder ticks
    private int inchesToTicks(double heightInches) {
        if (heightInches < MIN_HEIGHT || heightInches > MAX_HEIGHT) {
            throw new IllegalArgumentException("Height out of range: " + heightInches);
        }
        return (int) (MIN_TICKS + ((heightInches - MIN_HEIGHT) / (MAX_HEIGHT - MIN_HEIGHT)) * (MAX_TICKS - MIN_TICKS));
    }

    // Convert encoder ticks to inches
    private double ticksToInches(int ticks) {
        if (ticks < MIN_TICKS || ticks > MAX_TICKS) {
            throw new IllegalArgumentException("Ticks out of range: " + ticks);
        }
        return MIN_HEIGHT + ((double) (ticks - MIN_TICKS) / (MAX_TICKS - MIN_TICKS)) * (MAX_HEIGHT - MIN_HEIGHT);
    }

    // Action to move slides to a specific height in inches
    public Action moveSlidesToHeightAction(double targetHeight, double power) {
        return new Action() {
            private boolean initialized = false;
            private boolean done = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    int targetTicks = inchesToTicks(targetHeight); // Convert height to ticks

                    slideLeft.setTargetPosition(targetTicks);
                    slideRight.setTargetPosition(targetTicks);

                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    slideLeft.setPower(power);
                    slideRight.setPower(power);

                    initialized = true;
                }

                // Monitor motor status
                if (!done) {
                    done = !slideLeft.isBusy() && !slideRight.isBusy();
                }

                // Add telemetry for debugging
                packet.put("Slide Left Position (Ticks)", slideLeft.getCurrentPosition());
                packet.put("Slide Right Position (Ticks)", slideRight.getCurrentPosition());
                packet.put("Slide Left Position (Inches)", ticksToInches(slideLeft.getCurrentPosition()));
                packet.put("Slide Right Position (Inches)", ticksToInches(slideRight.getCurrentPosition()));
                packet.put("Target Height (Inches)", targetHeight);
                packet.put("Slide Action Done", done);

                return !done; // Keep running until the slides reach their target
            }
        };
    }
}
