package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import java.util.Arrays;
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

public class  ShooterSystem {



    public DcMotorEx shooterLeft, shooterRight;

    private DcMotor intake;
    private CRServo feederServo, feederServo2;
    private Servo Kicker1, Kicker2;
    public AbsoluteAnalogEncoder spindexerEncoder, AnglerEncoder;
    public CRServoEx spindexer, Angler;
    private AnalogInput servoEncoder;
    private NormalizedColorSensor colorSensor, colorSensor2;
    public CRServo anglerServo;              // raw CRServo
    private HoodServoController hood;
    public double optimalAngle1 = 0, optimalAngle2 = 0, optimalAngle3 = 0;
    public double[] optimalAnglesSERVO = new double[3];

    public double hoodDeg;

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
        //feederServo2 = hardwareMap.get(CRServo.class, "feederServo2"); // (0)

        Kicker1 = hardwareMap.get(Servo.class, "Kicker1");             // servo port 1
        Kicker2 = hardwareMap.get(Servo.class, "Kicker2");             // servo port 3
        AnglerEncoder = new AbsoluteAnalogEncoder(hardwareMap, "servoEncoder", 3.3, AngleUnit.DEGREES);
        anglerServo   = hardwareMap.get(CRServo.class, "feederServo"); // the hood CR servo itself

        hood = new HoodServoController(
                anglerServo,
                AnglerEncoder,
                new PIDFCoefficients(0.007, 0.0, 1, 0.0), // start simple; D usually 0 at first
                8,   // min hood deg
                220    // max hood deg (treated as "end stop", not wrap)
        );
        /*AnglerEncoder = new AbsoluteAnalogEncoder(hardwareMap, "servoEncoder", 3.3, AngleUnit.DEGREES);
        Angler = new CRServoEx(hardwareMap, "feederServo", AnglerEncoder, CRServoEx.RunMode.OptimizedPositionalControl);*/
        spindexerEncoder = new AbsoluteAnalogEncoder(hardwareMap, "MelonEncoder1", 3.3, AngleUnit.RADIANS);
        spindexer = new CRServoEx(hardwareMap, "SpindexerServo", spindexerEncoder, CRServoEx.RunMode.OptimizedPositionalControl);
        //CRServoEx spindexer = new CRServoEx(hardwareMap, "s_crServoEx", spindexerEncoder, CRServoEx.RunMode.OptimizedPositionalControl);
        spindexer.setPIDF(new PIDFCoefficients(0.275, 0.0, 1, 0));
        //.5
        spindexer.setCachingTolerance(.0002);
        /*Angler.setPIDF(new PIDFCoefficients(.01, 0.0, 1, 0.0001));
        Angler.setCachingTolerance(.0002);*/
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "spindexerColorSensor");
        colorSensor.setGain(12);

        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "spindexerColorSensor2");
        colorSensor2.setGain(12);
    }
//spindexer
// degrees
// degrees
private final double[] SHOOT_ANGLES  = { 81.5 ,201 ,320.5};
    private final double[] INTAKE_ANGLES = {21, 140, 261};

    private int shootIndex  = 0;
    private int intakeIndex = 0;

    public double targetRad = Math.toRadians(201);
    public double targetangler = 15;
    // edge state (store here so it can't get duplicated)
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;

    public void handleSpindexerButtons(boolean a, boolean b) {
        if (a && !prevA) {
            KickerDown();
            nextIntakeSlot();// or nextShootSlot(), your choice
        }
        if (b && !prevB) {
            KickerDown();
            nextShootSlot();
        }
        prevA = a;
        prevB = b;
    }

    public void nextShootSlot() {
        shootIndex = (shootIndex + 1) % SHOOT_ANGLES.length;
        setTargetDeg(SHOOT_ANGLES[shootIndex]);
    }

    public void nextIntakeSlot() {
        intakeIndex = (intakeIndex + 1) % INTAKE_ANGLES.length;
        setTargetDeg(INTAKE_ANGLES[intakeIndex]);
    }

    private void setTargetDeg(double deg) {
        targetRad = Math.toRadians(deg);
    }

    public void updateSpindexer() {
        spindexer.set(targetRad); // call every loop
    }
    //Hood Angler
    public void updateAngler() {
        //Angler.set(targetangler); // call every loop
        hood.update();
    }
    public void setTargetangler(double deg) {
        targetangler = deg;
        hood.setTargetDeg(targetangler);
    }

    void updateAutoShot(double distanceInches) {
        // --- RPM zone with hysteresis ---
        if (distanceInches > 105) this.setShooterTargetRpm(4200);;
        if (distanceInches < 95) this.setShooterTargetRpm(3400);;


        // --- Hood mapping ---
        hoodDeg = mapClamp(distanceInches, 0, 120, 20, 220.0);
        this.setTargetangler(hoodDeg);
    }

    static double mapClamp(double d, double dMin, double dMax, double aMin, double aMax) {
        double t = (d - dMin) / (dMax - dMin);
        t = Range.clip(t, 0.0, 1.0);
        return aMin + t * (aMax - aMin);
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
        Kicker2.setPosition(0.5);
    }

    public void KickerDown() {
        Kicker2.setPosition(1);}

    // rightTrigger -> forward, leftTrigger -> reverse
    public void controlFeeder(double rightTrigger, double leftTrigger) {
        double servoPower = rightTrigger - leftTrigger; // combine both triggers

        feederServo.setPower(-servoPower);
    }

    //   ---------------- ANGLERS ----------------



    //   ---------------- SERVO INDEXING ----------------

    public boolean spinningServo = false;

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
