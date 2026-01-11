package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Simple Mecanum Drive with Shooter", group = "TeleOp")
public class Teleopp extends LinearOpMode {

    // ---------------- DRIVE ----------------
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    // ---------------- SHOOTER ----------------
    private ShooterSystem shooterSystem;

    // ---------------- LIMELIGHT ----------------
    private Limelight3A limelight;
    private LLResult ll;

    // ---------------- ANALOG INPUTS ----------------
    private AnalogInput analogEncoder, servoEncoder;

    // ---------------- AIM MODE ----------------
    // ---- AutoShot toggle ----
    private boolean autoShotEnabled = false;
    private boolean prevX = false;

    // ---- DriveTrue toggle ----
    private boolean drivetrue = false;
    private boolean prevG2Y = false;

    // ---- Manual shooter controls ----
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    // ---- Angler trim ----
    private boolean prevG2A = false;
    private boolean prevG2B = false;

    private boolean aimMode = false;
    private boolean prevBack = false;

    // Aim PD constants
    private static final double AIM_KP = 0.02;
    private static final double AIM_KD = 0.00011;
    private static final double AIM_MAX_TURN = 0.75;
    private static final double TX_TOL_DEG = 0.7;
    private static final double MIN_TURN = 0.085;

    // D state
    private double lastTx = 0.0;
    private long lastAimTimeMs = 0;

    // ---------------- AUTO SHOT ----------------
    private boolean prevStart = false;

    private double filteredDistance = 0;
    private static final double DIST_ALPHA = 0.2;

    // ---------------- INTAKE / SHOOTER TOGGLES ----------------
    private boolean shooterHigh = false;
    private boolean shooterLow = false;
    private boolean intakeFwd = false, prevLB = false;
    private boolean intakeRev = false, prevRB = false;

    // ---------------- KICKER ----------------
    private boolean kickerActive = false;
    private long kickerStartTime = 0;
    private static final long KICKER_TIME_MS = 300;

    private boolean prevY = false;

    @Override
    public void runOpMode() {

        // -------- Hardware map --------
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        shooterSystem = new ShooterSystem(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        analogEncoder = hardwareMap.get(AnalogInput.class, "MelonEncoder1");
        servoEncoder  = hardwareMap.get(AnalogInput.class, "servoEncoder");

        shooterSystem.KickerDown();

        telemetry.addLine("Ready.");
        telemetry.addLine("Robot-centric always.");
        telemetry.addLine("BACK = toggle Aim Mode (Tx -> 0).");
        telemetry.update();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {

            // ---------------- Subsystems ----------------
            shooterSystem.handleSpindexerButtons(gamepad1.a, gamepad1.b);
            shooterSystem.updateSpindexer();
            shooterSystem.updateAngler();
            shooterSystem.updateShooterBangBang();

            // ---------------- AutoShot toggle ----------------
            boolean startNow = gamepad1.x;
            if (startNow && !prevStart) {
                autoShotEnabled = !autoShotEnabled;
            }

            prevStart = startNow;
// ---------------- DriveTrue toggle (gamepad2.y) ----------------
            boolean g2y = gamepad2.y;
            if (g2y && !prevG2Y) {
                drivetrue = !drivetrue;
            }
            prevG2Y = g2y;

            if (autoShotEnabled) {
                ll = limelight.getLatestResult();
                if (ll != null && ll.isValid()) {
                    double rawDistance =
                            31.84 / Math.sin(Math.toRadians(ll.getTy() + 15));

                    filteredDistance =
                            filteredDistance * (1 - DIST_ALPHA) +
                                    rawDistance * DIST_ALPHA;

                    shooterSystem.updateAutoShot(filteredDistance);
                }
            } else if (drivetrue&!autoShotEnabled) {
                boolean dpadLeft = gamepad1.dpad_left;
                boolean dpadRight = gamepad1.dpad_right;

                if (dpadLeft && !prevDpadLeft) {
                    shooterSystem.setShooterTargetRpm(3250);
                }

                if (dpadRight && !prevDpadRight) {
                    shooterSystem.setShooterTargetRpm(4250);
                }

                prevDpadLeft = dpadLeft;
                prevDpadRight = dpadRight;

                // ---- Angler trim ----
                boolean g2b = gamepad2.b;
                boolean g2a = gamepad2.a;

                if (g2b && !prevG2B) {
                    shooterSystem.setTargetangler(shooterSystem.hoodDeg + 10);
                }

                if (g2a && !prevG2A) {
                    shooterSystem.setTargetangler(shooterSystem.hoodDeg - 10);
                }

                prevG2A = g2a;
                prevG2B = g2b;
            }
            else {
                shooterSystem.stopShooterBangBang();
            }

            // ---------------- Aim mode toggle ----------------
            boolean back = gamepad2.back;
            if (back && !prevBack) {
                aimMode = !aimMode;
            }
            prevBack = back;

            // ---------------- Drive input ----------------
            double y  = gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            ll = limelight.getLatestResult();
            double turn = rx;

            // ---------------- PD Aim ----------------
            if (aimMode && ll != null && ll.isValid()) {

                double tx = ll.getTx();

                if (Math.abs(tx) <= TX_TOL_DEG) {
                    turn = 0.0;
                } else {
                    long now = System.currentTimeMillis();
                    double dt = (now - lastAimTimeMs) / 1000.0;
                    if (dt <= 0) dt = 0.02;

                    double dTx = (tx - lastTx) / dt;

                    double out = (AIM_KP * tx) - (AIM_KD * dTx);
                    out = Range.clip(out, -AIM_MAX_TURN, AIM_MAX_TURN);

                    if (Math.abs(out) < MIN_TURN) {
                        out = Math.copySign(MIN_TURN, tx);

                    }

                    turn = out;
                    lastTx = tx;
                    lastAimTimeMs = now;
                }
            }

            // ---------------- Mecanum ----------------
            double fl = y + x + turn;
            double bl = y - x + turn;
            double fr = y - x - turn;
            double br = y + x - turn;

            double max = Math.max(Math.abs(fl),
                    Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));

            if (max > 1.0) {
                fl /= max;
                bl /= max;
                fr /= max;
                br /= max;
            }

            leftFront.setPower(fl);
            leftRear.setPower(bl);
            rightFront.setPower(fr);
            rightRear.setPower(br);

            // ---------------- Intake ----------------
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            if (lb && !prevLB) {
                intakeFwd = !intakeFwd;
                if (intakeFwd) shooterSystem.intakeOn(1.0);
                else shooterSystem.intakeOff();
            }
            prevLB = lb;

            if (rb && !prevRB) {
                intakeRev = !intakeRev;
                if (intakeRev) shooterSystem.intakeReverse(1.0);
                else shooterSystem.intakeOff();
            }
            prevRB = rb;

            // ---------------- Kicker ----------------
            boolean yNow = gamepad1.y;

            if (yNow && !prevY && !kickerActive) {
                shooterSystem.KickerUp();
                kickerStartTime = System.currentTimeMillis();
                kickerActive = true;
            }

            if (kickerActive &&
                    System.currentTimeMillis() - kickerStartTime >= KICKER_TIME_MS) {
                shooterSystem.KickerDown();
                kickerActive = false;
            }
            prevY = yNow;

            // ---------------- Telemetry ----------------
            telemetry.addData("AimMode", aimMode);
            telemetry.addData("AutoShot", autoShotEnabled);

            if (ll != null) {
                telemetry.addData("LL valid", ll.isValid());
                telemetry.addData("Tx", ll.getTx());
                telemetry.addData("Ty", ll.getTy());
                telemetry.addData("Ta", ll.getTa());

                if (ll.isValid()) {
                    double distance =
                            31.84 / Math.sin(Math.toRadians(ll.getTy() + 15));
                    telemetry.addData("Distance", distance);
                }
            }

            double ang = shooterSystem.spindexerEncoder.getCurrentPosition();
            telemetry.addData("Spindexer Target (rad)", shooterSystem.targetRad);
            telemetry.addData("Spindexer Angle (rad)", ang);

            double melonDeg = (analogEncoder.getVoltage() / 3.3) * 360.0;
            double servoDeg = (servoEncoder.getVoltage() / 3.3) * 360.0;

            telemetry.addData("MelonEnc (deg)", melonDeg);
            telemetry.addData("ServoEnc (deg)", servoDeg);
            telemetry.addData("HoodDeg", shooterSystem.hoodDeg);
            telemetry.addData("Shooter Target RPM", shooterSystem.getShooterTargetRpm());
            telemetry.addData("Shooter RPM", shooterSystem.getShooterRpm());

            telemetry.update();
        }
    }
}
