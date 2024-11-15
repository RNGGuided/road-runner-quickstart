package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import static java.lang.Thread.sleep;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Mecanum Drive with Corrected Viper Slides", group = "TeleOp")
public class MainOpMode extends OpMode {

    // Declare motor objects for mecanum drive
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // Declare motor objects for Viper Slides
    private DcMotor slideLeft, slideRight;

    // Servo Wrist
    private Servo wristRight, wristLeft, claw;


    @Override
    public void init() {
        // Initialize motors from the hardware map (Mecanum Drive)
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Initialize motors from the hardware map (Viper Slides)
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");
        wristRight = hardwareMap.get(Servo.class, "wristRight");
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        claw = hardwareMap.get(Servo.class, "claw");
        // Set motor directions (Reverse right side motors for mecanum drive)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set initial power to zero for mecanum drive motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Set initial power to zero for slide motors
        slideLeft.setPower(0);
        slideRight.setPower(0);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.setPosition(1);
    }

    @Override
    public void loop() {
        // --- Mecanum Drive Controls ---
        double forwardBackward = -gamepad1.left_stick_y; // Forward/backward (Y axis of left stick)
        double strafe = gamepad1.left_stick_x;           // Strafe left/right (X axis of left stick)
        double rotate = gamepad1.right_stick_x;          // Rotation (X axis of right stick)

        // Calculate the power for each wheel
        double leftFrontPower = (forwardBackward + strafe + rotate);
        double rightFrontPower = (forwardBackward - strafe - rotate);
        double leftBackPower = (forwardBackward - strafe + rotate);
        double rightBackPower = (forwardBackward + strafe - rotate);

        // Normalize the values to make sure no wheel power exceeds 1.0
        double max = Math.max(1.0, Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

        leftFrontPower /= max;
        rightFrontPower /= max;
        leftBackPower /= max;
        rightBackPower /= max;

        // Apply the power to each motor
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        // --- Viper Slide Controls ---
        // Control 1st Viper Slide (using triggers)
        if (gamepad1.right_trigger > 0.1) {
            slideLeft.setPower(gamepad1.right_trigger); // Right trigger moves both slides up
            slideRight.setPower(-gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1) {
            slideLeft.setPower(-gamepad1.left_trigger); // Left trigger moves both slides down
            slideRight.setPower(gamepad1.left_trigger);
        } else {
            slideLeft.setPower(-0.1); // Freeze slides when no input
            slideRight.setPower(-0.1);
        }
        if (gamepad1.right_bumper)
        {
            claw.setPosition(1);
        }
        else if (gamepad1.left_bumper)
        {
            claw.setPosition(0);
        }
        if (gamepad1.a) {
            claw.setPosition(1);
            wristRight.setPosition(0);
            wristLeft.setPosition(0);
        } else if (gamepad1.b) {
            claw.setPosition(1);

            // Wait for 0.1 seconds
            try {
                Thread.sleep(100); // 100 milliseconds = 0.1 seconds
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            wristRight.setPosition(0.5);
            wristLeft.setPosition(0.5);
        }
        else if (gamepad1.x) {
            claw.setPosition(1);

            try {
                Thread.sleep(100); // 100 milliseconds = 0.1 seconds
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            wristRight.setPosition(1);
            wristLeft.setPosition(1);
        }

        else if (gamepad1.y) {
            claw.setPosition(1);
            wristRight.setPosition(0.9);
            wristRight.setPosition(0.9);
        }
        // --- Telemetry for Debugging ---
        telemetry.addData("Left Front Power", leftFrontPower);
        telemetry.addData("Right Front Power", rightFrontPower);
        telemetry.addData("Left Back Power", leftBackPower);
        telemetry.addData("Right Back Power", rightBackPower);
        telemetry.addData("Slide Left Power", slideLeft.getPower());
        telemetry.addData("Slide Right Power", slideRight.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors when the OpMode stops
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        slideLeft.setPower(0);
        slideRight.setPower(0);
    }
}
