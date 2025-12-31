package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class    ShooterSystem {



    public DcMotorEx shooterLeft, shooterRight;

    private DcMotor intake;
    private CRServo feederServo, feederServo2;
    private Servo Kicker1, Kicker2;
    public AbsoluteAnalogEncoder spindexerEncoder;
    public CRServoEx spindexer;
    private AnalogInput servoEncoder;
    private NormalizedColorSensor colorSensor, colorSensor2;

    public double optimalAngle1 = 0, optimalAngle2 = 0, optimalAngle3 = 0;
    public double[] optimalAnglesSERVO = new double[3];

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public ShooterSystem(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
// port 3
        intake = hardwareMap.get(DcMotor.class, "intake");             // port 0

        feederServo = hardwareMap.get(CRServo.class, "feederServo");   // servo port 4
        feederServo2 = hardwareMap.get(CRServo.class, "feederServo2"); // (0)

        Kicker1 = hardwareMap.get(Servo.class, "Kicker1");             // servo port 1
        Kicker2 = hardwareMap.get(Servo.class, "Kicker2");             // servo port 3

        //AbsoluteAnalogEncoder spindexerEncoder = new AbsoluteAnalogEncoder(hardwareMap, "MelonEncoder1");
        spindexerEncoder = new AbsoluteAnalogEncoder(hardwareMap, "MelonEncoder1");
        spindexer = new CRServoEx(hardwareMap, "SpindexerServo", spindexerEncoder, CRServoEx.RunMode.OptimizedPositionalControl);
        //CRServoEx spindexer = new CRServoEx(hardwareMap, "s_crServoEx", spindexerEncoder, CRServoEx.RunMode.OptimizedPositionalControl);
        spindexer.setPIDF(new PIDFCoefficients(0.8, 0.0, 0.1, 0.0001));
        spindexer.setCachingTolerance(0.0002);
        servoEncoder = hardwareMap.get(AnalogInput.class, "servoEncoder");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "spindexerColorSensor");
        colorSensor.setGain(12);

        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "spindexerColorSensor2");
        colorSensor2.setGain(12);
    }
//spindexer
// degrees
    private final double[] shootAngles  = {29.34, 149.23, 269.5636};
    private final double[] intakeAngles = {88.36, 212.4, 330.65 };


    private int shootIndex  = 0;
    private int intakeIndex = 0;

    private double currentTargetDeg = 0;
    private boolean spindexerBusy = false;
    private static final double SPINDEX_TOL_DEG = .5;
    public void nextShootSlot() {
        shootIndex = (shootIndex + 1) % 3;
        setShootSlot(shootIndex);
    }

    public void prevShootSlot() {
        shootIndex = (shootIndex + 2) % 3;
        setShootSlot(shootIndex);
    }

    public void setShootSlot(int slot) {
        slot = Math.max(0, Math.min(2, slot));
        currentTargetDeg = shootAngles[slot];
        spindexer.set(Math.toRadians(currentTargetDeg)); // SolversLib handles shortest path + PIDF
    }

    public void nextIntakeSlot() {
        intakeIndex = (intakeIndex + 1) % 3;
        setIntakeSlot(intakeIndex);
    }

    public void prevIntakeSlot() {
        intakeIndex = (intakeIndex + 2) % 3;
        setIntakeSlot(intakeIndex);
    }

    public void setIntakeSlot(int slot) {
        slot = Math.max(0, Math.min(2, slot));
        currentTargetDeg = intakeAngles[slot];
        spindexer.set(Math.toRadians(currentTargetDeg));
    }

    public void spin1() {
        spindexer.set(Math.toRadians(88.36));
    }
    public boolean isSpindexerBusy() {
        return spindexerBusy;
    }

    //color
    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();

        float normRed  = colors.red   / colors.alpha;
        float normGreen= colors.green / colors.alpha;
        float normBlue = colors.blue  / colors.alpha;

        float normRed2  = colors2.red   / colors2.alpha;
        float normGreen2= colors2.green / colors2.alpha;
        float normBlue2 = colors2.blue  / colors2.alpha;

        // Only log if telemetry is NOT null
        /*if (telemetry != null) {
            telemetry.addData("red", normRed);
            telemetry.addData("Green", normGreen);
            telemetry.addData("Blue", normBlue);
            telemetry.addData("red2", normRed2);
            telemetry.addData("Green2", normGreen2);
            telemetry.addData("Blue2", normBlue2);
        }*/

        // GREEN thresholds
        if ((normRed < 0.2 && normGreen > 0.6 && normBlue < 0.6) ||
                (normRed2 < 0.3 && normGreen2 > 0.8 && normBlue2 < 0.75)) {
            return DetectedColor.GREEN;
        }

        // PURPLE thresholds
        if ((normRed > 0.25 && normGreen < 0.5 && normBlue > 0.55) ||
                (normRed2 > 0.45 && normGreen2 < 0.6 && normBlue2 > 0.8)) {
            return DetectedColor.PURPLE;
        }

        return DetectedColor.UNKNOWN;
    }


    public int containsGreen(int[] array) {
        for (int i = 0; i < 3; i++) {
            if (array[i] == 1) {
                return i;
            }
        }
        return -1;
    }

    public int containsPurple(int[] array) {
        for (int i = 0; i < 3; i++) {
            if (array[i] == 2) {
                return i;
            }
        }
        return -1;
    }

    // ------------ Shooter + Intake ------------

    public void shoot(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    public void stopShooter() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    public void intakeOn(double power) {
        intake.setPower(-power);
    }

    public void intakeOff() {
        intake.setPower(0.0);
    }

    public void intakeReverse(double power) {
        intake.setPower(power);
    }

    public void KickerUp() {
        Kicker1.setPosition(0);
        Kicker2.setPosition(0.5);
    }

    public void KickerDown() {
        Kicker1.setPosition(0.5);
        Kicker2.setPosition(0);
    }

    // rightTrigger -> forward, leftTrigger -> reverse
    public void controlFeeder(double rightTrigger, double leftTrigger) {
        double servoPower = rightTrigger - leftTrigger; // combine both triggers

        feederServo2.setPower(servoPower);
        feederServo.setPower(-servoPower);
    }

    //   ---------------- ANGLERS ----------------

    int initCycles = 0;
    boolean optimalSet = false;

    public void getOptimalAngles() {
        if (optimalSet) return;

        double voltage2 = servoEncoder.getVoltage();
        double angle2 = (voltage2 / 3.3) * 360.0;

        if (initCycles < 5) { // wait a few cycles
            initCycles++;
            return;
        }

        optimalAngle1 = angle2;
        optimalAngle2 = (angle2 + 40) % 360;
        optimalAngle3 = (angle2 + 240) % 360;
        optimalSet = true;

        optimalAnglesSERVO[0] = (optimalAngle1 + 3) % 360;
        optimalAnglesSERVO[1] = optimalAngle1;
        optimalAnglesSERVO[2] = (optimalAngle1 - 3 + 360) % 360;
    }

    //   ---------------- SERVO INDEXING ----------------

    boolean spinningServo = false;
    double targetAngleServo = 0;
    final double TOLERANCESERVO = 2; // smaller tolerance since PID is smoother

    int currentTargetIndexServo = -1;
    int directionServo = 1;

    double kP = 0.08;
    double kI = 0.000;
    double kD = 0.001;

    double integralServo = 0;
    double lastErrorServo = 0;
    long lastTimeServo = 0;

    public void spinToNextAngleServo(double currentAngle) {
        if (spinningServo) return;

        if (currentTargetIndexServo == -1) {
            currentTargetIndexServo = 0;
        } else {
            currentTargetIndexServo += directionServo;

            if (currentTargetIndexServo >= optimalAnglesSERVO.length) {
                currentTargetIndexServo = optimalAnglesSERVO.length - 2;
                directionServo = -1;
            } else if (currentTargetIndexServo < 0) {
                currentTargetIndexServo = 1;
                directionServo = 1;
            }
        }

        targetAngleServo = optimalAnglesSERVO[currentTargetIndexServo];
        spinningServo = true;

        integralServo = 0;
        lastErrorServo = 0;
        lastTimeServo = System.currentTimeMillis();
    }

    public void updateServo() {
        if (!spinningServo) return;

        double voltage2 = servoEncoder.getVoltage();
        double angle2 = (voltage2 / 3.3) * 360.0;

        double error = targetAngleServo - angle2;
        error = ((error + 540) % 360) - 180;

        long now = System.currentTimeMillis();
        double deltaTime = (now - lastTimeServo) / 1000.0;
        if (deltaTime <= 0) deltaTime = 0.001;

        integralServo += error * deltaTime;
        double derivative = (error - lastErrorServo) / deltaTime;

        double power = kP * error + kI * integralServo + kD * derivative;

        power = Math.max(-0.3, Math.min(0.3, power));

        feederServo.setPower(power);
        feederServo2.setPower(-power);

        lastErrorServo = error;
        lastTimeServo = now;

        if (Math.abs(error) < TOLERANCESERVO && Math.abs(power) < 0.5) {
            feederServo.setPower(0);
            feederServo2.setPower(0);
            spinningServo = false;
        }
    }
// ======================================================
//                 BANG-BANG SHOOTER CONTROL
// ======================================================

    // ======================================================
//                 SHOOTER CONTROL (MODES)
// ======================================================



    public enum ShooterControlMode {

        BANG_BANG,
        HYBRID,
        PIDF
    }

    public ShooterControlMode shooterMode = ShooterControlMode.HYBRID;

    // Target + filtered RPM
    private double shooterTargetRpm = 0;
    private double shooterFilteredRpm = 0;

    // 0 = raw, 1 = very smoothed. 0.2 is good for FTC shooters.
    private static final double SHOOTER_SMOOTHING = 0.2;

    // Bang-bang / hybrid behavior
    private static final double RPM_TOLERANCE   = 10;   // when "close enough" for pure bang-bang
    private static final double HYBRID_BAND_RPM = 250;  // how far below target we use full send

    // PIDF constants for shooter
// *** YOU WILL TUNE THESE ***
    private double shooterKp = 0.0008;   // proportional gain
    private double shooterKd = 0.0002;      // start at 0, add later if needed
    private double shooterKf = 0.000225;   // feedforward term (base power per RPM)

    private double lastShooterError = 0;

    /** Return the target shooter RPM */
    public double getShooterTargetRpm() {
        return shooterTargetRpm;
    }

    /** Choose which control mode to use */
    public void setShooterMode(ShooterControlMode mode) {
        shooterMode = mode;
    }

    /** Set the shooter target RPM */
    public void setShooterTargetRpm(double rpm) {
        shooterTargetRpm = rpm;
    }

    /** Read shooter velocity (averages both motors) */
    public double getShooterRpm() {
        // Adjust if your motor encoder CPR is not 28.
        double ticksPerRev = 28;

        double vL = shooterLeft.getVelocity();  // ticks/sec
        //double vR = shooterRight.getVelocity(); // ticks/sec

        double rpmL = (vL / ticksPerRev) * 60.0;
        //double rpmR = (vR / ticksPerRev) * 60.0;

        return rpmL;
    }

    public double getShooterVelocity() {
        return shooterLeft.getVelocity();
    }
    public double getCurrentPosition() {
        return shooterLeft.getCurrentPosition();
    }
    /** Main shooter control update – call EVERY LOOP (TeleOp + auton) */
    private void updateShooterControl() {
        double currentRpm = getShooterRpm();

        // Exponential smoothing filter
        shooterFilteredRpm =
                shooterFilteredRpm * (1.0 - SHOOTER_SMOOTHING) +
                        currentRpm * SHOOTER_SMOOTHING;

        // No target? Turn off shooter.
        if (shooterTargetRpm <= 0) {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
            return;
        }

        double error = shooterTargetRpm - shooterFilteredRpm;
        double derivative = error - lastShooterError;
        lastShooterError = error;

        double power;

        switch (shooterMode) {
            case BANG_BANG:
                // Classic: below target -> full power, above -> off
                if (shooterFilteredRpm < shooterTargetRpm - RPM_TOLERANCE) {
                    power = 1.0;
                } else {
                    power = 0.0;
                }
                break;

            case HYBRID:
                // Far below target -> bang-bang full send
                if (shooterFilteredRpm < shooterTargetRpm - HYBRID_BAND_RPM) {
                    power = 1.0;
                } else {
                    // Close to target -> Feedforward + P (+ optional D later)
                    // base feedforward: proportional to target RPM
                    double ff = shooterKf * shooterTargetRpm;
                    double pTerm = shooterKp * error;
                    double dTerm = shooterKd * derivative;  // start with 0

                    power = ff + pTerm + dTerm;
                }
                break;

            case PIDF:
            default:
                // Always use FF + PID, no pure bang-bang phase
                double ffPid = shooterKf * shooterTargetRpm;
                double pTermPid = shooterKp * error;
                double dTermPid = shooterKd * derivative;  // add if needed

                power = ffPid + pTermPid + dTermPid;
                break;
        }

        // Clip power to valid range
        power = Range.clip(power, 0.0, 1.0);

        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    /** Wrapper so your existing code still compiles */
    public void updateShooterBangBang() {
        updateShooterControl();
    }

    /** Stop shooter completely */
    public void stopShooterBangBang() {
        shooterTargetRpm = 0;
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    /** Helper: are we close enough to speed? */
    public boolean atShooterSpeed() {
        return shooterTargetRpm > 0 &&
                Math.abs(shooterTargetRpm - shooterFilteredRpm) < RPM_TOLERANCE;
    }


    /** Main shooter control update – call EVERY LOOP (TeleOp + auton) */



    public boolean isSpinningServo() {
        return spinningServo;
    }
}
