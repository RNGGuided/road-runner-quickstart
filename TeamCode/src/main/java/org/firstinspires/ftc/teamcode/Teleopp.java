package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;




@TeleOp(name = "Simple Mecanum Drive with Shooter", group = "TeleOp")
public class Teleopp extends LinearOpMode {


    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private ShooterSystem shooterSystem;


    AnalogInput analogEncoder, servoEncoder;




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


        analogEncoder = hardwareMap.get(AnalogInput.class, "MelonEncoder1");
        servoEncoder = hardwareMap.get(AnalogInput.class, "servoEncoder");


        boolean shooter = false;
        boolean prevX = false;
        boolean intake = false;
        boolean prevD_pad = false;
        boolean shooter2 = false;
        boolean prevA = false;
        boolean prevD_padL = false;
        boolean intake2 = false;










        waitForStart();


        while (opModeIsActive()) {
            shooterSystem.getOptimalAngles();
            shooterSystem.updateServo();


            boolean currentX = gamepad1.x;
            boolean currentD_pad = gamepad1.dpad_up;
            boolean currentA = gamepad1.a;
            boolean currentD_padL = gamepad1.dpad_left;
            // ---------------- DRIVE ----------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            double frontLeftPower  = y + x + rx;
            double backLeftPower   = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower  = y + x - rx;


            // ---------CR SERVO (LAZY) ----------
            boolean currentDpadRight = gamepad1.dpad_right;


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


            double voltage = analogEncoder.getVoltage(); // get voltage (0V–3.3V)
            double angle = (voltage / 3.3) * 360.0;


            double voltage2 = servoEncoder.getVoltage(); // get voltage (0V–3.3V)
            double servoAngle = (voltage2 / 3.3) * 360.0;




            // ---------------- SHOOTER ----------------
           /*
           if (gamepad1.a) {
               shooterSystem.shoot(0.99);  // Full power when A is pressed
           } else {
               shooterSystem.stopShooter(); // Stop when released
           }
           */


            if (currentA && !prevA) {
                shooter2 = !shooter2;
                if (shooter2) shooterSystem.shoot(0.5175);
                else shooterSystem.stopShooter();
            }

// Shooter toggle X (0.65 power)
            if (currentX && !prevX) {
                shooter = !shooter;
                if (shooter) shooterSystem.shoot(0.62);
                else shooterSystem.stopShooter();
            }

// ✅ Intake Forward on RIGHT BUMPER
            if (gamepad1.right_bumper) {
                shooterSystem.intakeOn(1.0);
            } else if (!gamepad1.left_bumper) { // only stop if left bumper not pressed
                shooterSystem.intakeOff();
            }

// ✅ Intake Reverse on LEFT BUMPER
            /*if (gamepad1.left_bumper) {
                shooterSystem.intakeReverse(1.0);
            } else if (!gamepad1.right_bumper) { // only stop if right bumper not pressed
                shooterSystem.intakeOff();
            }
*/
            if (gamepad1.b) {
                shooterSystem.KickerUp();  // Pushes the kicker Up when B is pressed
            }

            if(gamepad1.y) {
                shooterSystem.KickerDown(); // Puts them down when y is pressed
            }


            if (gamepad1.dpad_right && !shooterSystem.isSpinning()) {
                shooterSystem.KickerUp();
                shooterSystem.spinToNextAngle(angle);
            }
            shooterSystem.update();


           /*if (gamepad1.b)
           {
               shooterSystem.KickerUp();
           }
           */


            /*if (shooterSystem.optimalSet) { // ✅ only allow spinning after angles are set
                if (gamepad1.b && !shooterSystem.isSpinningServo()) {
                    shooterSystem.spinToNextAngleServo(servoAngle);
                }
            } else {
                telemetry.addLine("⏳ Waiting for optimal angles...");
            }*/








            // Continuously update servo movement






            // When you press D-Pad Right, start spinning if not already
           /*if (gamepad1.dpad_right && !spinning && !prevDpadRight && !cooldown) {
               shooterSystem.controlSpindexer(); // starts spinning
               servoTimer.reset();                // start the timer
               spinning = true;                   // mark as spinning
           }


           // Stop spinning after 2 seconds
           if (spinning && servoTimer.seconds() > 0.33333) {
               shooterSystem.stopSpindexer();     // stop the servo
               spinning = false;                  // mark as stopped
               cooldown = true;
               cooldownTimer.reset();


           }


           if (cooldown && cooldownTimer.seconds() > 1) {
               cooldown = false;
           }
           */


            prevX = currentX;
            prevD_pad = currentD_pad;
            prevA = currentA;
            prevD_padL = currentD_padL;
            // ---------------- FEEDER SERVO ----------------
            shooterSystem.controlFeeder(gamepad1.right_trigger, gamepad1.left_trigger);


            telemetry.addData("OptimalAngle1", shooterSystem.optimalAngle1);
            telemetry.addData("OptimalAngle2", shooterSystem.optimalAngle2);
            telemetry.addData("OptimalAngle3", shooterSystem.optimalAngle3);
            telemetry.addData("Volts", voltage2);
            telemetry.addData("Servo Angle", servoAngle);
            telemetry.addData("Spindexer Angle", angle);
            telemetry.addData("Target Servo", shooterSystem.targetAngleServo);
            telemetry.addData("Target", shooterSystem.targetAngle);
            telemetry.addData("Diff", (shooterSystem.targetAngle - angle + 360) % 360);
            telemetry.addData("Spinning", shooterSystem.isSpinning());
            telemetry.addData("Angle (deg)", angle);
            telemetry.addData("Encoder Voltage", voltage);
            telemetry.addData("Shooter", gamepad1.a ? "Active" : "Off");
            telemetry.addData("Feeder Power", gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("Intake", gamepad1.dpad_up ? "Active" : "Off");
            telemetry.update();
        }
    }
}

