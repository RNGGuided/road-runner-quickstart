package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    private DcMotorEx slideLeft, slideRight;

    public Slides(HardwareMap hardwareMap) {
        // Initialize slide motors
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        // Set directions (adjust based on your hardware setup)
        slideLeft.setDirection(DcMotor.Direction.FORWARD);
        slideRight.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to use encoders
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveSlidesToPosition(int targetPosition, double power) {
        // Set target position
        slideLeft.setTargetPosition(targetPosition);
        slideRight.setTargetPosition(targetPosition);

        // Set motors to RUN_TO_POSITION
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power
        slideLeft.setPower(power);
        slideRight.setPower(power);

        // Wait until the slides reach the position
        while (slideLeft.isBusy() || slideRight.isBusy()) {
            // Optional: Add telemetry or logging here
        }

        // Stop the motors
        slideLeft.setPower(0);
        slideRight.setPower(0);

        // Reset to RUN_USING_ENCODER
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
