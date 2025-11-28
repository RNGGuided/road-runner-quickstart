package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ShooterSystem;


@TeleOp(name = "Simple Mecanum Drive with Shooter", group = "TeleOp")
public class Teleopp extends LinearOpMode {


    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private ShooterSystem shooterSystem;


    AnalogInput analogEncoder, servoEncoder;


    public int[] spindexerBalls = new int[3]; // 1 = green, 2 = purple
    private int spindexerBallIndex = -1, ballAmount = 0;
    long kickerTimer = 0;






    @Override
    public void runOpMode() {
        // Drive hardware map
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");




        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        // Shooter system setup
        shooterSystem = new ShooterSystem(hardwareMap);


        ShooterSystem.DetectedColor detectedColor;






        analogEncoder = hardwareMap.get(AnalogInput.class, "MelonEncoder1");
        servoEncoder = hardwareMap.get(AnalogInput.class, "servoEncoder");


        boolean shooter = false, prevDpad_Left = false, intake = false, prevLBumper = false, shooter2 = false, prevDpad_right = false, prevRBumper = false, intake2 = false, prevY = false, kicker = false, shootingKicker = false;


        shooterSystem.KickerUp();


        waitForStart();


        while (opModeIsActive()) {
            shooterSystem.updateShooterBangBang();
            shooterSystem.getOptimalAngles();
            shooterSystem.updateServo();


            // --------------- Switch Machine Var --------------
            boolean currentDpad_left = gamepad1.dpad_left;
            boolean currentLBumper = gamepad1.left_bumper;
            boolean currentDpad_right = gamepad1.dpad_right;
            boolean currentRBumper = gamepad1.right_bumper;
            boolean currentY = gamepad1.y;
            // ---------------- DRIVE ----------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            double frontLeftPower  = y + x + rx;
            double backLeftPower   = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower  = y + x - rx;




            double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower),
                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
            if (max > 1.0) {
                frontLeftPower  /= max;
                backLeftPower   /= max;
                frontRightPower /= max;
                backRightPower  /= max;
            }


            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);


            //   ---------------- ANGLES ----------------


            double voltage = analogEncoder.getVoltage(); // get voltage (0V–3.3V)
            double angle = (voltage / 3.3) * 360.0;


            double voltage2 = servoEncoder.getVoltage(); // get voltage (0V–3.3V)
            double servoAngle = (voltage2 / 3.3) * 360.0;




            if(shootingKicker && !shooterSystem.isSpinning() && !shooterSystem.isSpinning2())
            {
                shooterSystem.KickerDown();
                shootingKicker = false;
            }
            // ---------------- SHOOTER ----------------


            if (currentDpad_left && !prevDpad_Left) {
                shooter2 = !shooter2;
                if (shooter2) {
                    shooterSystem.setShooterTargetRpm(1675);  // also 2500 or change it
                } else {
                    shooterSystem.stopShooterBangBang();
                }
            }



            if (currentDpad_right && !prevDpad_right) {
                shooter = !shooter;
                if (shooter) {
                    shooterSystem.setShooterTargetRpm(2050);  // 🔥 YOUR RPM HERE
                } else {
                    shooterSystem.stopShooterBangBang();
                }
            }


            // ---------------- INTAKE ----------------


            if (currentLBumper && !prevLBumper) {
                intake = !intake;
                if(intake) {
                    shooterSystem.intakeOn(1.0);
                }
                else {
                    shooterSystem.intakeOff();
                }
            }


            if (currentRBumper && !prevRBumper) {
                intake2 = !intake2;
                if(intake2) {
                    shooterSystem.intakeReverse(1.0);
                }
                else {
                    shooterSystem.intakeOff();
                }
            }

            // ---------------- KICKER ----------------


            if(currentY && !prevY) {
                kicker = !kicker;
                if(kicker) {
                    shooterSystem.KickerDown(); // Puts them down when y is pressed
                }
                else {
                    shooterSystem.KickerUp();
                }
            }


            //  ---------------- SPINDEXER ----------------


            if (gamepad1.dpad_up && !shooterSystem.isSpinning()) {
                shooterSystem.spinToNextAngle();
            }
            shooterSystem.update();


            if (gamepad1.dpad_down && !shooterSystem.isSpinning2()) {
                shooterSystem.spinToNextAngle2();
            }
            shooterSystem.update2();


            // Shoots a Green ball (SHOOTER MUST ALREADY BE ACTIVE)
            if(gamepad1.x && !shooterSystem.isSpinning() && !shooterSystem.isSpinning2() && ballAmount != 0 && shooterSystem.containsGreen(spindexerBalls) != -1 && shooterSystem.currentTargetIndex != -1)
            {
                int rotation = shooterSystem.containsGreen(spindexerBalls) - shooterSystem.currentTargetIndex;
                if(rotation == 0)
                {
                    shooterSystem.KickerDown();
                }
                else if (rotation == -1 || rotation == 2)
                {
                    shooterSystem.spinToNextAngle();
                    shootingKicker = true;
                    kickerTimer = System.currentTimeMillis();
                }
                else if (rotation == 1 || rotation == -2)
                {
                    shooterSystem.spinToNextAngle2();
                    shootingKicker = true;
                    kickerTimer = System.currentTimeMillis();
                }


                ballAmount--;
                spindexerBalls[shooterSystem.containsGreen(spindexerBalls)] = 0;


            }


            // Shoots a Purple ball (SHOOTER MUST ALREADY BE ACTIVE)
            if(gamepad1.b && !shooterSystem.isSpinning() && !shooterSystem.isSpinning2() && ballAmount != 0 && shooterSystem.containsPurple(spindexerBalls) != -1 && shooterSystem.currentTargetIndex != -1)
            {
                int rotation = shooterSystem.containsPurple(spindexerBalls) - shooterSystem.currentTargetIndex;
                if(rotation == 0)
                {
                    shooterSystem.KickerDown();
                }
                else if (rotation == -1 || rotation == 2)
                {
                    shooterSystem.spinToNextAngle();
                    shootingKicker = true;
                    kickerTimer = System.currentTimeMillis();
                }
                else if (rotation == 1 || rotation == -2)
                {
                    shooterSystem.spinToNextAngle2();
                    shootingKicker = true;
                    kickerTimer = System.currentTimeMillis();
                }


                ballAmount--;
                spindexerBalls[shooterSystem.currentTargetIndex] = 0;


            }


            //  ---------------- COLOR SENSORS ----------------


            detectedColor = shooterSystem.getDetectedColor(telemetry);


            if(detectedColor != ShooterSystem.DetectedColor.UNKNOWN && ballAmount != 3 && !shooterSystem.isSpinning2() && !shooterSystem.isSpinning())
            {
                spindexerBallIndex = shooterSystem.currentTargetIndex;


                if(spindexerBallIndex == -1)
                {
                    spindexerBallIndex = 0;
                }
                if(detectedColor == ShooterSystem.DetectedColor.GREEN && spindexerBalls[spindexerBallIndex] != 1)
                {
                    spindexerBalls[spindexerBallIndex] = 1;
                    ballAmount = (ballAmount + 1) % 4;
                }
                else if (detectedColor == ShooterSystem.DetectedColor.PURPLE && spindexerBalls[spindexerBallIndex] != 2)
                {
                    spindexerBalls[spindexerBallIndex] = 2;
                    ballAmount = (ballAmount + 1) % 4;
                }
            }
            // ---------------- STATE MACHINE ----------------


            prevDpad_Left = currentDpad_left;
            prevLBumper = currentLBumper;
            prevDpad_right = currentDpad_right;
            prevRBumper = currentRBumper;
            prevY = currentY;


            // ---------------- FEEDER SERVO ----------------
            if (gamepad2.y) shooterSystem.setShooterMode(ShooterSystem.ShooterControlMode.HYBRID);
            if (gamepad2.x) shooterSystem.setShooterMode(ShooterSystem.ShooterControlMode.PIDF);
            if (gamepad2.b) shooterSystem.setShooterMode(ShooterSystem.ShooterControlMode.BANG_BANG);



            shooterSystem.controlFeeder(gamepad1.right_trigger, gamepad1.left_trigger);


            //   ---------------- TELEMETRY ----------------
            telemetry.addData("Target RPM", shooterSystem.getShooterTargetRpm());
            telemetry.addData("RPM Raw", shooterSystem.getShooterRpm());
            telemetry.addData("Shooter Speed OK", shooterSystem.atShooterSpeed());
            telemetry.addData("Mode", shooterSystem.shooterMode); // if you want mode display
            telemetry.update();
            telemetry.addData("Shooter RPM", shooterSystem.getShooterRpm());
            telemetry.addData("ContainsPurple", shooterSystem.containsPurple(spindexerBalls));
            telemetry.addData("spindexerBalls1", spindexerBalls[0]);
            telemetry.addData("spindexerBalls2", spindexerBalls[1]);
            telemetry.addData("spindexerBalls3", spindexerBalls[2]);
            telemetry.addData("INDEX", spindexerBallIndex);
            telemetry.addData("CURRENTTARGETINDEX", shooterSystem.currentTargetIndex);
            telemetry.addData("Ball Amount", ballAmount);
            telemetry.addData("Detected Color", detectedColor);


            //telemetry.addData("Target2", shooterSystem.targetAngle2);
            //telemetry.addData("Target", shooterSystem.targetAngle);
            //telemetry.addData("Diff", (shooterSystem.targetAngle - angle + 360) % 360);
            //telemetry.addData("Angle (deg)", angle);
            telemetry.update();
        }
    }
}
