package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Slides Move to 26.5 (FTC SDK)", group = "Test")
public class Slides1 extends LinearOpMode {

    private DcMotorEx slideLeft, slideRight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize slides motors
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        // Reset and configure encoders
        slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Calculate target position for 26.5 inches (assuming 0 = 16 inches, 4400 ticks = 51.5 inches)
        int targetTicks = (int) ((26.5 - 16) / (51.5 - 16) * 4400);

        // Set target positions
        slideLeft.setTargetPosition(targetTicks);
        slideRight.setTargetPosition(targetTicks);

        // Set power for the slides
        slideLeft.setPower(1.0);
        slideRight.setPower(1.0);

        telemetry.addLine("Slides Initialized. Ready to Start.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Let the slides move while program continues
        while (opModeIsActive() && (slideLeft.isBusy() || slideRight.isBusy())) {
            telemetry.addData("Slide Left Position", slideLeft.getCurrentPosition());
            telemetry.addData("Slide Right Position", slideRight.getCurrentPosition());
            telemetry.addData("Target Position", targetTicks);
            telemetry.update();
        }

        // Stop the motors once the target is reached
        slideLeft.setPower(0);
        slideRight.setPower(0);

        telemetry.addLine("Slides moved to 26.5 inches. Program continuing...");
        telemetry.update();

        // Add additional logic here, the program keeps running
        while (opModeIsActive()) {
            telemetry.addLine("Program running...");
            telemetry.update();
        }
    }
}
