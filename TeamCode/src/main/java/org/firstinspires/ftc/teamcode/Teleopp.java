package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoAimConfig.*;
import static org.firstinspires.ftc.teamcode.AutoAimConfig.AIM_KD;
import static org.firstinspires.ftc.teamcode.AutoAimConfig.AIM_KP;
import static org.firstinspires.ftc.teamcode.AutoAimConfig.AIM_MAX_TURN;
import static org.firstinspires.ftc.teamcode.AutoAimConfig.MIN_TURN;
import static org.firstinspires.ftc.teamcode.AutoAimConfig.TX_TOL_DEG;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.Arrays;

@Config

@TeleOp(name = "Simple Mecanum Drive with Shooter", group = "TeleOp")
public class Teleopp extends LinearOpMode {
    // ---------------- DRIVE ----------------
    private DcMotor leftFront, rightFront, leftRear, rightRear;
    // ---------------- SHOOTER ----------------
    private ShooterSystem shooterSystem;
    // ---------------- LIMELIGHT ----------------
    private Servo spindexerServo,Husky, spindexerServo2, spindexerServo3;
    private Limelight3A limelight;
    private LLResult ll;

    private DigitalChannel limitSwitch;

    private int[] cachedObeliskPattern = null;
    private int cachedObeliskTag = -1;

    public boolean ballCurrentlyDetected = false;
    public boolean previousBallDetected = false;
    public int lastDetectedColor = -1;
    private long lastIndexTime = 0;
    private static final long INDEX_COOLDOWN_MS = 300; // tune (200–500ms)
    private boolean waitingForBurstKick = false;
    private long burstKickTimer = 0;
    private static final long BURST_KICK_DELAY = 275; // ms

    // ---------------- ANALOG INPUTS ----------------
    private AnalogInput analogEncoder, servoEncoder;
    // ---------------- COLOR SORTING ---------------
    public Integer[] spindexerBalls = {0, 0, 0}; // 1 = green, 2 = purple
    private int spindexerBallIndex = -1, ballAmount = 0;
    long kickerTimer = 0;

    HuskyLens husky;
    private int color, width, height, color2, width2, height2;
    int motif = 0;
    private boolean patternActive = false;
    private int[] pattern;          // 1 = purple, 2 = green
    private int patternIndex = 0;
    boolean kickerDown;
    //1100
    private static final long PATTERN_DELAY_MS =700;
    private long lastPatternStepTime = 0;
    private boolean waitingForPatternDelay = false;
    // ---------------- AIM MODE ----------------
    // ---- AutoShot toggle ----
    private boolean autoShotEnabled = false;
    private boolean prevX = false;
    // ---- DriveTrue toggle ----
    private boolean drivetrue = false;
    private boolean prevG2Y = false;

    private int guaranteedShotsRemaining = 0;
    // ---- Manual shooter controls ----
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    // ---- Angler trim ----
    private boolean prevG2A = false;
    private boolean prevG2B = false, prevG2U, prevG2D, prevG2X;
    private int shotsRemaining = 0;


    // ---- Cached Obelisk Pattern ----
    private int cachedTagId = -1;
    private boolean autoAimAllowed = false;
    int qPos = 0;
    double rotationTime = 0, rotationTime2;
    boolean rotate, rotate2, spin, spin2, rotate3;
    int SPINTIME = 300;
    private boolean gppActive = false;
    private int gppStep = 0;   // 0 = G, 1 = P, 2 = P
    private boolean aimMode = false;
    private boolean prevBack = false;
    // Aim PD constants
    // D state
    private double lastTx = 0.0;
    private boolean autoRotateAfterKick = false;
    private long lastAimTimeMs = 0;
    // ---------------- AUTO SHOT ----------------
    private boolean prevStart = false, prevA, prevB;
    private double filteredDistance = 0, pos = 0.516;
    private static final double DIST_ALPHA = 0.2;
    private double LONG_WAIT = 600;
    final double[] SHOOT_POS  = {0.105, 0.225, 0.347};
    final double[] INTAKE_POS = {0.17, 0.286, 0.41};
    Double[] spinAngles = {238.4727, 349.8545, 120.2182, 2.0727};
    int shootIndex  = 0;
    int intakeIndex = 0;
    private boolean next;
    // ---------------- INTAKE / SHOOTER TOGGLES ----------------
    private boolean shooterHigh = false, purple, green;
    private boolean shooterLow = false;
    private boolean intakeFwd = false, prevLB = false;
    private boolean intakeRev = false, prevRB = false, prevG2BUM;
    // ---------------- KICKER ----------------
    private boolean kickerActive = false, ColorKickerActive = false;
    private long kickerStartTime = 0, ColorKicker = 0;
    private static final long KICKER_TIME_MS = 120
            , COLORKICKERTIME = 340;
    private int ballCount;

    int detectedColor = 0;

    private boolean prevY = false;
    private boolean justIndexed;
    private boolean autoShootActive = false;
    private int autoShootStep = 0;
    private long spindexerTimer = 0;
    private boolean waitingForSpin = false;
    private long spinDelayStart = 0;
    private static final long SPIN_DELAY_AFTER_KICK = 250; // ms
    private boolean waitingForSpindexer = false;
    private static final long SPINDEXER_DELAY = 250; // time between rotations
    private boolean huskyActive = false;
    private long huskyTimer = 0;
    private static final long HUSKY_DELAY = 100;
    private boolean allowNewBall = true;
    boolean pressed;
    // extra delay before 3rd shot
    private static final long THIRD_SHOT_EXTRA_DELAY = 200; // tune (80–200)
    private boolean waitingForThirdShot = false;
    // -------- SAFE BURST SYSTEM (SLOW VERSION) --------
    private boolean safeWaitingForBurstKick = false;
    private long safeBurstKickTimer = 0;
    private static final long SAFE_BURST_KICK_DELAY = 400; // slower

    private boolean readyToIndex = true;
    private int safeShotsRemaining = 0;
    private boolean safeWaitingForSpin = false;
    private long safeSpinDelayStart = 0;

    private boolean safeWaitingForSpindexer = false;

    private static final long SAFE_SPIN_DELAY_AFTER_KICK = 250; // slower
    private static final long SAFE_SPINDEXER_DELAY = 175;       // slower
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
        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");
        limelight.pipelineSwitch(0);
        //analogEncoder = hardwareMap.get(AnalogInput.class, "MelonEncoder1");
        servoEncoder  = hardwareMap.get(AnalogInput.class, "servoEncoder");
        shooterSystem.KickerDown();
        telemetry.addLine("Ready.");
        telemetry.addLine("Robot-centric always.");
        telemetry.addLine("BACK = toggle Aim Mode (Tx -> 0).");
        telemetry.update();
        husky = hardwareMap.get(HuskyLens.class, "husky");
        husky.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        spindexerServo = hardwareMap.get(Servo.class, "spindexerServo");
        spindexerServo2 = hardwareMap.get(Servo.class, "spindexerServo2");
        spindexerServo3 = hardwareMap.get(Servo.class, "spindexerServo3");
        Husky = hardwareMap.get(Servo.class,"Huskyservo");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        //AnalogInput encoder = hardwareMap.get(AnalogInput.class, "MelonEncoder1");
        //RTPAxon servo = new RTPAxon(crservo, encoder);

        //shooterSystem.setTargetangler(150);
        // 0.516,
        //.33
        //shooterSystem.setIntakeIndex(0);
        waitForStart();
        limelight.start();
        Husky.setPosition(1);
        //shooterSystem.setIntakeIndex(intakeIndex);
        //spin(INTAKE_POS[0]);
        while (opModeIsActive()) {
            // ---------------- Subsystems -----------
            shooterSystem.updateShooterBangBang();
            shooterSystem.updateAngler();
            //shooterSystem.spin.update();   // ⭐ ADD THIS LINE
            updateObeliskFromLimelight();
            // ---------------- HUSKY ----------------------
            HuskyLens.Block[] blocks = husky.blocks();
            pressed = !limitSwitch.getState();
            //telemetry.addData("Block count", blocks.length);
            //double melonDeg = (analogEncoder.getVoltage() / 3.3)*360.0;

            //leftRear.setPower(1);
            //spindexerServo1 off by 0.01
            //spindexerServo.setPosition(0.092);
            //spindexerServo2.setPosition(0.102);
            //spindexerServo3.setPosition(0.102);


            //0.17, 0.286, 0.41

            //0,


            //shoot 0.098,0.218,0.34,0.465,.592,.71, 0.829, 0.96

            int pastIntakeIndex = -1;
            int pastColor = -1;

            // ---------- BALL DETECTION ----------
            ballCurrentlyDetected = false;

            if (blocks.length != 0 && !huskyActive) {
                color = blocks[0].id;
                width = blocks[0].width;
                height = blocks[0].height;

                color2 = (blocks.length >= 2) ? blocks[1].id : -1;
                width2 = (blocks.length >= 2) ? blocks[1].width : -1;
                height2 = (blocks.length >= 2) ? blocks[1].height : -1;

                if (width * height > width2 * height2 && color != 0) {
                    detectedColor = color;
                    ballCurrentlyDetected = true;
                }
                else if (width2 * height2 > width * height && color2 != 0) {
                    detectedColor = color2;
                    ballCurrentlyDetected = true;
                }
                boolean spindexerFull =
                        spindexerBalls[0] != 0 &&
                                spindexerBalls[1] != 0 &&
                                spindexerBalls[2] != 0;

                if (spindexerFull && ballCurrentlyDetected) {
                    spindexerBalls[intakeIndex] = detectedColor;
                }
            }

// ---------- NEW BALL CHECK (EDGE DETECTION) ----------
            boolean spindexerFull =
                    spindexerBalls[0] != 0 &&
                            spindexerBalls[1] != 0 &&
                            spindexerBalls[2] != 0;

            boolean newBall =
                    allowNewBall &&
                            readyToIndex &&
                            ballCurrentlyDetected &&
                            detectedColor != 0 &&
                            System.currentTimeMillis() - lastIndexTime > INDEX_COOLDOWN_MS &&
                            !spindexerFull;

            if (newBall) {
                spindexerBalls[intakeIndex] = detectedColor;

                intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                spin(INTAKE_POS[intakeIndex]);


                lastIndexTime = System.currentTimeMillis();
                readyToIndex = false;
            }



// reset when ball leaves view
            if (!ballCurrentlyDetected) {
                readyToIndex = true;
                lastDetectedColor = -1;
            }

// update previous state
            previousBallDetected = ballCurrentlyDetected;

            if(gamepad1.a && !prevA)
            {
                shootIndex = (shootIndex + 1) % SHOOT_POS.length;
                spin(SHOOT_POS[shootIndex]);

            }
            prevA = gamepad1.a;

            if(gamepad1.b && !prevB)
            {
                huskyTimer = System.currentTimeMillis();
                huskyActive = true;

                spin(INTAKE_POS[(intakeIndex + 1) % INTAKE_POS.length]);

                intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
            }
            prevB = gamepad1.b;

            ElapsedTime kickerRest = new ElapsedTime();
            //telemetry.addData("ContainsP?", Arrays.asList(spindexerBalls).indexOf(1));

            if(ColorKickerActive &&
                    System.currentTimeMillis() - ColorKicker >= COLORKICKERTIME)
            {
                shooterSystem.KickerUp();
                ColorKickerActive = false;
                kickerActive = true;
                kickerStartTime = System.currentTimeMillis();
            }

            if(huskyActive &&
                    System.currentTimeMillis() - huskyTimer >= HUSKY_DELAY)
            {// return down
                huskyActive = false;
            }

            boolean autoNow = gamepad1.dpad_up;


            if(autoNow && !prevG2U){
                threeBallBurst();
            }

            prevG2U = autoNow;

            if(autoShootActive){

                long now = System.currentTimeMillis();

                // wait until kicker finishes and delay passes
                if(!kickerActive && now - kickerStartTime >= KICKER_TIME_MS + SPIN_DELAY_AFTER_KICK){

                    autoShootStep++;

                    if(autoShootStep > 2){
                        autoShootActive = false;
                        allowNewBall = true;

                        Arrays.fill(spindexerBalls, 0);
                        intakeIndex = 0;
                    }
                    else{

                        // rotate to next position
                        shootIndex = autoShootStep;
                        spin(SHOOT_POS[shootIndex]);

                        // trigger next kick
                        shooterSystem.KickerUp();
                        kickerStartTime = now;
                        kickerActive = true;
                    }
                }
            }
                   /*
                   else if((ColorKickerActive &&
                           System.currentTimeMillis() - LONG_WAIT >= COLORKICKERTIME && (Math.abs(spinAngles[0] - melonDeg) < 1) || Math.abs(spinAngles[1] - melonDeg) < 1 || Math.abs(spinAngles[2] - melonDeg) < 1 || Math.abs(spinAngles[3] - melonDeg) < 1))
                   {
                       shooterSystem.KickerUp();
                       ColorKickerActive = false;
                       kickerActive = true;
                       kickerStartTime = System.currentTimeMillis();
                   }
                    */
            // ---------------- AutoShot toggle ----------------
            // ---------------- AutoShot toggle (gamepad1.x) ----------------
            boolean xNow = gamepad1.x;

            if (xNow && !prevX) {
                autoShotEnabled = !autoShotEnabled;
            }

            prevX = xNow;


// ---------------- Aim Mode toggle (gamepad2.start) ----------------
            boolean startNow = gamepad2.start;

            if (startNow && !prevStart) {
                aimMode = !aimMode;
            }

            prevStart = startNow;
            // ---------------- DriveTrue toggle (gamepad2.y) ----------------
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
            }
            else {
                shooterSystem.stopShooterBangBang();
            }
            // ---------------- Aim mode toggle ----------------
            /*boolean back = gamepad2.back;
            if (back && !prevBack) {
                aimMode = !aimMode;
            }
            prevBack = back;*/
            // ---------------- Drive input ----------------
            double y  = gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            ll = limelight.getLatestResult();
            double turn = rx;
            // ---------------- PD Aim ----------------
            double tx = Double.NaN;

            if (aimMode && ll != null && ll.isValid()) {

                for (LLResultTypes.FiducialResult fid : ll.getFiducialResults()) {

                    int id = fid.getFiducialId();

                    if (id == 20 || id == 24) {

                        tx = fid.getTargetXDegrees();

                        // ✅ EXTRA OFFSET ONLY FOR TAG 24
                        if (id == 24) {
                            tx += 1.3;   // rotate 0.5° more to the right
                        }
                        if (id == 20) {
                            tx -=    2.3 ;   // rotate 0.5° more to the right
                        }

                        break;
                    }
                }

            }

            if (Double.isNaN(tx)) {
                turn = rx;     // no allowed tag → manual drive
            }
            else if (Math.abs(tx) <= TX_TOL_DEG) {
                turn = 0.0;
            }
            else {
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

                if(shotsRemaining > 0){

                    shotsRemaining--;

                    if(shotsRemaining > 0){

                        // start delay AFTER kicker down
                        spinDelayStart = System.currentTimeMillis();
                        waitingForSpin = true;
                    }
                    else {

                        // ⭐ restore intake indexing after final shot
                        allowNewBall = true;

                    }


                }
            }
            // delay AFTER kick down
            long requiredDelay = SPIN_DELAY_AFTER_KICK;

// if we're about to shoot the third ball, wait longer
            if(shotsRemaining == 1){
                requiredDelay += THIRD_SHOT_EXTRA_DELAY;
            }

            if(waitingForSpin &&
                    System.currentTimeMillis() - spinDelayStart >= requiredDelay &&
                    !kickerActive){

                waitingForSpin = false;

                shootIndex++;

                if(shootIndex < SHOOT_POS.length){
                    spin(SHOOT_POS[shootIndex]);
                }

                // now wait for spindexer to physically rotate
                spinDelayStart = System.currentTimeMillis();
                waitingForSpindexer = true;
            }

            if(waitingForBurstKick &&
                    System.currentTimeMillis() - burstKickTimer >= BURST_KICK_DELAY){

                waitingForBurstKick = false;

                shoot();
            }

// delay AFTER spinning before next kick
            if(waitingForSpindexer &&
                    System.currentTimeMillis() - spinDelayStart >= SPINDEXER_DELAY){

                waitingForSpindexer = false;

                if(shotsRemaining > 0){
                    shoot();
                }
            }
            prevY = yNow;

            // -------- SAFE BURST LOGIC --------
            if(safeWaitingForBurstKick &&
                    System.currentTimeMillis() - safeBurstKickTimer >= SAFE_BURST_KICK_DELAY){

                safeWaitingForBurstKick = false;

                shootSafe();
            }

            if(safeWaitingForSpin &&
                    System.currentTimeMillis() - safeSpinDelayStart >= SAFE_SPIN_DELAY_AFTER_KICK &&
                    !kickerActive){

                safeWaitingForSpin = false;

                shootIndex++;

                if(shootIndex < SHOOT_POS.length){
                    spin(SHOOT_POS[shootIndex]);
                }

                safeSpinDelayStart = System.currentTimeMillis();
                safeWaitingForSpindexer = true;
            }

            if(safeWaitingForSpindexer &&
                    System.currentTimeMillis() - safeSpinDelayStart >= SAFE_SPINDEXER_DELAY){

                safeWaitingForSpindexer = false;

                if(safeShotsRemaining > 0){
                    shootSafe();
                }
            }
            if(gamepad1.dpad_left){
                threeBallBurstSafe();
            }

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

            //double ang = shooterSystem.spindexerEncoder.getVoltage();
            //telemetry.addData("Spindexer Target (rad)", shooterSystem.targetRad);
            //telemetry.addData("Spindexer Angle", Math.toDegrees(ang));
            double servoDeg = (servoEncoder.getVoltage() / 3.3) * 360.0;

            //telemetry.addData("ServoEnc (deg)", servoDeg);
            telemetry.addData("HoodDeg", shooterSystem.hoodDeg);
            telemetry.addData("Shooter Target RPM", shooterSystem.getShooterTargetRpm());
            telemetry.addData("Shooter RPM", shooterSystem.getShooterRpm());
            telemetry.addData("Hood Target", shooterSystem.getHoodTargetPos());
            telemetry.addData("Spindexer1", spindexerBalls[0]);
            telemetry.addData("Spindexer2", spindexerBalls[1]);
            telemetry.addData("Spindexer3", spindexerBalls[2]);
            telemetry.addData("intakeIndex", intakeIndex);
            telemetry.addData("shootIndex", shootIndex);
            telemetry.addData("color", color);
            telemetry.addData("color2", color2);
            telemetry.addData("blocks length", blocks.length);
            telemetry.addData("ColoredKicker", kickerActive);
            telemetry.addData("Obelisk Tag", cachedObeliskTag);
            telemetry.addData("Obelisk Pattern",
                    cachedObeliskPattern == null ? "NONE" : Arrays.toString(cachedObeliskPattern));
            telemetry.addData("BallAmount", ballAmount);
            //telemetry.addData("Shoot Pos", SHOOT_POS[shootIndex]);
            telemetry.addData("newBall?", newBall);
            telemetry.addData("Color", color);
            telemetry.addData("Husky Active", huskyActive);
            telemetry.addData("pressed?", pressed);
            //telemetry.addData("intakeIndex", intakeIndex);
            telemetry.update();
        }
    }

    private void updateObeliskFromLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {

            int id = fiducial.getFiducialId();

            // Lock once so it doesn't change mid-match
            if (cachedObeliskPattern != null) return;

            switch (id) {
                case 21:
                    cachedObeliskPattern = new int[]{2,1,1}; // GPP
                    cachedObeliskTag = 21;
                    break;

                case 22:
                    cachedObeliskPattern = new int[]{1,2,1}; // PGP
                    cachedObeliskTag = 22;
                    break;

                case 23:
                    cachedObeliskPattern = new int[]{1,1,2}; // PPG
                    cachedObeliskTag = 23;
                    break;
            }
        }
    }

    public void spin(double value)
    {
        spindexerServo.setPosition(value-0.01);
        spindexerServo2.setPosition(value);
        spindexerServo3.setPosition(value);
    }
    public void shoot() {

        shooterSystem.KickerUp();
        kickerStartTime = System.currentTimeMillis();
        kickerActive = true;

        // mark that when kicker finishes we should rotate
        autoRotateAfterKick = true;
    }
    public void threeBallBurst(){

        allowNewBall = false;

        Arrays.fill(spindexerBalls, 0);

        shootIndex = 0;
        spin(SHOOT_POS[0]);

        shotsRemaining = 3;

        // start delay before kicking
        burstKickTimer = System.currentTimeMillis();
        waitingForBurstKick = true;

        previousBallDetected = false;
        lastDetectedColor = -1;
    }

    public void shootSafe() {

        shooterSystem.KickerUp();
        kickerStartTime = System.currentTimeMillis();
        kickerActive = true;

    }
    public void threeBallBurstSafe(){

        allowNewBall = false;

        Arrays.fill(spindexerBalls, 0);

        shootIndex = 0;
        spin(SHOOT_POS[0]);

        safeShotsRemaining = 3;

        safeBurstKickTimer = System.currentTimeMillis();
        safeWaitingForBurstKick = true;

        previousBallDetected = false;
        lastDetectedColor = -1;
    }

}

