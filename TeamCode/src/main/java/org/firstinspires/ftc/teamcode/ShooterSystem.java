package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;


public class ShooterSystem {


    private HuskyLens husky;

    public boolean ballCurrentlyDetected = false;
    public boolean previousBallDetected = false;

    public int detectedColor = 0;

    private int color, width, height;
    private int color2, width2, height2;

    private long lastIndexTime = 0;
    private static final long INDEX_COOLDOWN_MS = 300;

    public Integer[] spindexerBalls = {0,0,0};
    public DcMotorEx shooterLeft, shooterRight;

    // ---------- INDEXING ----------
    private boolean readyToIndex = true;
    private boolean allowNewBall = true;
    public int intakeIndex = 0;

    // ---------- QUICK SHOOT ----------
    private int shootIndex = 0;
    private int shotsRemaining = 0;

    private boolean kickerActive = false;
    private long kickerStartTime = 0;
    private static final long KICKER_TIME_MS = 105;

    private boolean waitingForSpin = false;
    private long spinDelayStart = 0;
    private static final long SPIN_DELAY_AFTER_KICK = 100;

    private boolean waitingForSpindexer = false;
    private static final long SPINDEXER_DELAY = 90;
    private DcMotor intake;
    private Servo Kicker1, Kicker2;
    public AbsoluteAnalogEncoder AnglerEncoder;
    public AnalogInput spindexerEncoder;
    // ---------------- SPINDEXER (POSITION SERVO) ----------------
    private Servo spindexerServo1,spindexerServo2,spindexerServo3;

    // MUST match TeleOp exactly
    final double[] SHOOT_POS  = {0.105, 0.225, 0.347};
    final double[] INTAKE_POS = {0.17, 0.286, 0.41};
    public Servo huskyServo;
    private AnalogInput servoEncoder;
    private NormalizedColorSensor colorSensor, colorSensor2;
    public Servo anglerServo;              // raw CRServo
    public HoodServoController hood;
    // --- Hood jitter control ---
    private double lastHoodPos = Double.NaN;
    private long lastHoodUpdateMs = 0;

    // Tunables
    private static final double HOOD_STEP = 0.01;          // servo quantization
    private static final long HOOD_UPDATE_MS = 60;         // rate limit


    private AnalogInput encoder;


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
        intake = hardwareMap.get(DcMotor.class, "intake");

        spindexerServo1  = hardwareMap.get(Servo.class, "spindexerServo");
        spindexerServo2 = hardwareMap.get(Servo.class, "spindexerServo2");
        spindexerServo3 = hardwareMap.get(Servo.class, "spindexerServo3");

        // port 0
        //spindexerEncoder = hardwareMap.get(AnalogInput.class, "MelonEncoder1");
        encoder = hardwareMap.get(AnalogInput.class, "servoEncoder");
        anglerServo = hardwareMap.get(Servo.class, "feederServo");

        husky = hardwareMap.get(HuskyLens.class, "husky");
        husky.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        AnglerEncoder = new AbsoluteAnalogEncoder(
                hardwareMap,
                "servoEncoder",
                3.3,
                AngleUnit.DEGREES
        );


        hood = new HoodServoController(
                anglerServo,
                0,   // absolute min
                1.0    // absolute max
        );


        //feederServo = hardwareMap.get(CRServo.class, "feederServo");   // servo port 4
        //feederServo2 = hardwareMap.get(CRServo.class, "feederServo2"); // (0)


        Kicker1 = hardwareMap.get(Servo.class, "Kicker1");             // servo port 1
        Kicker2 = hardwareMap.get(Servo.class, "Kicker2");             // servo port 3


        anglerServo = hardwareMap.get(Servo.class, "feederServo"); // the hood CR servo itself
        huskyServo = hardwareMap.get(Servo.class, "Huskyservo");




       /*AnglerEncoder = new AbsoluteAnalogEncoder(hardwareMap, "servoEncoder", 3.3, AngleUnit.DEGREES);
       Angler = new CRServoEx(hardwareMap, "feederServo", AnglerEncoder, CRServoEx.RunMode.OptimizedPositionalControl);*/
        //spindexer = new CRServoEx(hardwareMap, "SpindexerServo", spindexerEncoder, CRServoEx.RunMode.OptimizedPositionalControl);
        //CRServoEx spindexer = new CRServoEx(hardwareMap, "s_crServoEx", spindexerEncoder, CRServoEx.RunMode.OptimizedPositionalControl);
        //spindexer.setPIDF(new PIDFCoefficients(0.000, 1, 20, 1));
        //.5
        //spindexer.setCachingTolerance(.0002);
       /*Angler.setPIDF(new PIDFCoefficients(.01, 0.0, 1, 0.0001));
       Angler.setCachingTolerance(.0002);*/
        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "spindexerColorSensor");
        //colorSensor.setGain(12);


        //colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "spindexerColorSensor2");
        //colorSensor2.setGain(12);
    }

    //spindexer
// degrees
// degrees
    private static double quantize(double value, double step) {
        return Math.round(value / step) * step;
    }

    public double getHoodTargetPos() {
        return hood.getTargetPos();
    }


    public void stepIntakeSlot() {
        intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;

        spindexerServo1.setPosition(INTAKE_POS[intakeIndex] - 0.01);
        spindexerServo2.setPosition(INTAKE_POS[intakeIndex]);
        spindexerServo3.setPosition(INTAKE_POS[intakeIndex]);
    }


// inside ShooterSystem

    public void updateColorDetection()
    {
        HuskyLens.Block[] blocks = husky.blocks();

        ballCurrentlyDetected = false;

        if(blocks.length > 0)
        {
            color = blocks[0].id;
            width = blocks[0].width;
            height = blocks[0].height;

            color2 = (blocks.length >= 2) ? blocks[1].id : -1;
            width2 = (blocks.length >= 2) ? blocks[1].width : -1;
            height2 = (blocks.length >= 2) ? blocks[1].height : -1;

            if(width * height > width2 * height2 && color != 0)
            {
                detectedColor = color;
                ballCurrentlyDetected = true;
            }
            else if(width2 * height2 > width * height && color2 != 0)
            {
                detectedColor = color2;
                ballCurrentlyDetected = true;
            }
        }

        boolean spindexerFull =
                spindexerBalls[0] != 0 &&
                        spindexerBalls[1] != 0 &&
                        spindexerBalls[2] != 0;

        boolean newBall =
                allowNewBall &&
                        readyToIndex &&
                        ballCurrentlyDetected &&
                        !previousBallDetected &&
                        detectedColor != 0 &&
                        System.currentTimeMillis() - lastIndexTime > INDEX_COOLDOWN_MS &&
                        !spindexerFull;

        if(newBall)
        {
            spindexerBalls[intakeIndex] = detectedColor;

            intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
            setIntakeIndex(intakeIndex);

            lastIndexTime = System.currentTimeMillis();
            readyToIndex = false;
        }

        if(!ballCurrentlyDetected)
        {
            readyToIndex = true;
        }

        previousBallDetected = ballCurrentlyDetected;
    }

    public void startThreeBallBurst()
    {
        allowNewBall = false;

        Arrays.fill(spindexerBalls,0);

        shootIndex = 0;
        setShootIndex(0);

        shotsRemaining = 3;

        shoot();
    }

    public void shoot()
    {
        KickerUp();
        kickerStartTime = System.currentTimeMillis();
        kickerActive = true;
    }

    public void updateQuickShoot()
    {
        long now = System.currentTimeMillis();

        if(kickerActive && now - kickerStartTime >= KICKER_TIME_MS)
        {
            KickerDown();
            kickerActive = false;

            if(shotsRemaining > 0)
            {
                shotsRemaining--;

                if(shotsRemaining > 0)
                {
                    spinDelayStart = now;
                    waitingForSpin = true;
                }
                else
                {
                    allowNewBall = true;
                }
            }
        }

        if(waitingForSpin &&
                now - spinDelayStart >= SPIN_DELAY_AFTER_KICK &&
                !kickerActive)
        {
            waitingForSpin = false;

            shootIndex++;

            if(shootIndex < SHOOT_POS.length)
            {
                setShootIndex(shootIndex);
            }

            spinDelayStart = now;
            waitingForSpindexer = true;
        }

        if(waitingForSpindexer &&
                now - spinDelayStart >= SPINDEXER_DELAY)
        {
            waitingForSpindexer = false;

            if(shotsRemaining > 0)
            {
                shoot();
            }
        }
    }

    public void update()
    {
        updateShooterBangBang();
        updateAngler();
        updateColorDetection();
        updateQuickShoot();
    }
    public void huskyUp() {
        huskyServo.setPosition(0);
    }

    public void huskyDown() {
        huskyServo.setPosition(1);
    }

    public void stepShootSlot() {
        shootIndex = (shootIndex + 1) % SHOOT_POS.length;
        spindexerServo1.setPosition(SHOOT_POS[shootIndex]-.01);
        spindexerServo2.setPosition(SHOOT_POS[shootIndex]);
        spindexerServo3.setPosition(SHOOT_POS[shootIndex]);
    }

    public void setShootIndex(int index) {

        shootIndex = Math.floorMod(index, SHOOT_POS.length);
        spindexerServo1.setPosition(SHOOT_POS[shootIndex]-.01);
        spindexerServo2.setPosition(SHOOT_POS[shootIndex]);
        spindexerServo3.setPosition(SHOOT_POS[shootIndex]);

    }

    public void setIntakeIndex(int index) {

        intakeIndex = Math.floorMod(index, INTAKE_POS.length);
        spindexerServo1.setPosition(INTAKE_POS[intakeIndex]-.01);
        spindexerServo2.setPosition(INTAKE_POS[intakeIndex]);
        spindexerServo3.setPosition(INTAKE_POS[intakeIndex]);

    }


    public void updateSpindexer() {
        //spindexer.set(targetRad); // call every loop
    }

    //Hood Angler


    void  updateAutoShot(double distanceInches) {
        // --- RPM zone with hysteresis ---
        if (distanceInches > 105) this.setShooterTargetRpm(4350);
        ;
        if (distanceInches < 95) this.setShooterTargetRpm(3500);
        ;


        // --- Hood mapping ---
        // --- Hood mapping (INVERTED ON PURPOSE) ---
        double rawPos = mapClamp(
                distanceInches,
                5, 120,
                .9 ,   // close = down
                0.02      // far = up
        );

// Quantize to nearest 0.01
        double quantizedPos = quantize(rawPos, HOOD_STEP);

// Rate limit + change-only update
        long now = System.currentTimeMillis();
        if (now - lastHoodUpdateMs >= HOOD_UPDATE_MS &&
                (Double.isNaN(lastHoodPos) ||
                        Math.abs(quantizedPos - lastHoodPos) >= HOOD_STEP)) {

            setHoodPosition(quantizedPos);
            lastHoodPos = quantizedPos;
            lastHoodUpdateMs = now;
        }


    }

    public void setHoodPosition(double pos) {
        hood.setTargetPos(pos);
    }

    public void updateAngler() {
        hood.update();  // ← NOT servo.update()
    }


    static double mapClamp(double d, double dMin, double dMax, double aMin, double aMax) {
        double t = (d - dMin) / (dMax - dMin);
        t = Range.clip(t, 0.0, 1.0);
        return aMin + t * (aMax - aMin);
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
        Kicker1.setPosition(.5);
        Kicker2.setPosition(.5);
    }


    public void KickerDown() {
        Kicker1.setPosition(1);
        Kicker2.setPosition(0);
    }

    public boolean spinningServo = false;


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
    private static final double RPM_TOLERANCE = 10;   // when "close enough" for pure bang-bang
    private static final double HYBRID_BAND_RPM = 250;  // how far below target we use full send


    // PIDF constants for shooter
// *** YOU WILL TUNE THESE ***
    private double shooterKp = 0.0008;   // proportional gain
    private double shooterKd = 0.0002;      // start at 0, add later if needed
    private double shooterKf = 0.000225;   // feedforward term (base power per RPM)


    private double lastShooterError = 0;


    /**
     * Return the target shooter RPM
     */
    public double getShooterTargetRpm() {
        return shooterTargetRpm;
    }


    /**
     * Choose which control mode to use
     */
    public void setShooterMode(ShooterControlMode mode) {
        shooterMode = mode;
    }


    /**
     * Set the shooter target RPM
     */
    public void setShooterTargetRpm(double rpm) {
        shooterTargetRpm = rpm;
    }


    /**
     * Read shooter velocity (averages both motors)
     */
    public double getShooterRpm() {
        // Adjust if your motor encoder CPR is not 28.
        double ticksPerRev = 28;


        double vL = shooterLeft.getVelocity();  // ticks/sec
        //double vR = shooterRight.getVelocity(); // ticks/sec


        double rpmL = (vL / ticksPerRev) * 60.0;
        //double rpmR = (vR / ticksPerRev) * 60.0;


        return rpmL;
    }

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


    /**
     * Wrapper so your existing code still compiles
     */
    public void updateShooterBangBang() {
        updateShooterControl();
    }


    /**
     * Stop shooter completely
     */
    public void stopShooterBangBang() {
        shooterTargetRpm = 0;
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }


    /**
     * Helper: are we close enough to speed?
     */
    public boolean atShooterSpeed() {
        return shooterTargetRpm > 0 &&
                Math.abs(shooterTargetRpm - shooterFilteredRpm) < RPM_TOLERANCE;
    }
}