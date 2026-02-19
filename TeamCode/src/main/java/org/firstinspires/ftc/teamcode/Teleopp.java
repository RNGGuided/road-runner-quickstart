        package org.firstinspires.ftc.teamcode;







        import static org.firstinspires.ftc.teamcode.AutoAimConfig.*;
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
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;
        import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
        import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;








        import java.util.Arrays;
        import java.util.List;






        @Config

        @TeleOp(name = "Simple Mecanum Drive with Shooter", group = "TeleOp")
        public class Teleopp extends LinearOpMode {








            // ---------------- DRIVE ----------------
            private DcMotor leftFront, rightFront, leftRear, rightRear;








            // ---------------- SHOOTER ----------------
            private ShooterSystem shooterSystem;








            // ---------------- LIMELIGHT ----------------




            private Servo Posservo,Husky;
            private Limelight3A limelight;
            private LLResult ll;

            private int[] cachedObeliskPattern = null;
            private int cachedObeliskTag = -1;







            // ---------------- ANALOG INPUTS ----------------
            private AnalogInput analogEncoder, servoEncoder;








            // ---------------- COLOR SORTING ---------------








            public Integer[] spindexerBalls = new Integer[3]; // 1 = green, 2 = purple
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
            private long lastAimTimeMs = 0;








            // ---------------- AUTO SHOT ----------------
            private boolean prevStart = false, prevA, prevB;








            private double filteredDistance = 0, pos = 0.516;
            private static final double DIST_ALPHA = 0.2;




            private double LONG_WAIT = 600;








            final double[] SHOOT_POS  = {0.2, 0.575, 0.95};
            final double[] INTAKE_POS = {.15 ,.82, 0.49};
            Double[] spinAngles = {238.4727, 349.8545, 120.2182, 2.0727};
            int shootIndex  = 0;
            int intakeIndex = 0;


            private boolean next;








            // ---------------- INTAKE / SHOOTER TOGGLES ----------------
            private boolean shooterHigh = false, purple, green;
            private boolean shooterLow = false;
            private boolean intakeFwd = false, prevLB = false;
            private boolean intakeRev = false, prevRB = false, prevG2BUMP;








            // ---------------- KICKER ----------------
            private boolean kickerActive = false, ColorKickerActive = false;
            private long kickerStartTime = 0, ColorKicker = 0;
            private static final long KICKER_TIME_MS =150, COLORKICKERTIME = 340;






            private int ballCount;
            private boolean prevY = false;








            private boolean justIndexed;
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
















                husky = hardwareMap.get(HuskyLens.class, "husky");
                husky.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);







                telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
                //CRServo crservo = hardwareMap.crservo.get("SpindexerServo");
                //Posservo = hardwareMap.get(Servo.class, "SpindexerServo");
                Husky = hardwareMap.get(Servo.class,"Huskyservo");
                AnalogInput encoder = hardwareMap.get(AnalogInput.class, "MelonEncoder1");
                //RTPAxon servo = new RTPAxon(crservo, encoder);








                Husky.setPosition(1);
                //shooterSystem.setTargetangler(150);








                // 0.516,
                //.33
                shooterSystem.setIntakeIndex(0);
                waitForStart();
                limelight.start();
                shooterSystem.setIntakeIndex(intakeIndex);
                while (opModeIsActive()) {








                    // ---------------- Subsystems -----------
                    shooterSystem.updateShooterBangBang();
                    shooterSystem.updateAngler();
                    shooterSystem.spin.update();   // ⭐ ADD THIS LINE
                    updateObeliskFromLimelight();












                    // ---------------- HUSKY ----------------------








                    HuskyLens.Block[] blocks = husky.blocks();
                    //telemetry.addData("Block count", blocks.length);




                    double melonDeg = (analogEncoder.getVoltage() / 3.3)*360.0;


                    if(blocks.length != 0)
                    {
                        color = blocks[0].id;
                        width = blocks[0].width;
                        height = blocks[0].height;








                        color2 = (blocks.length >= 2) ? blocks[1].id : -1;
                        width2 = (blocks.length >= 2) ? blocks[1].width : -1;
                        height2 = (blocks.length >= 2) ? blocks[1].height : -1;








                        if(width > width2 && height > height2 && color != 0)
                        {
                            spindexerBalls[intakeIndex] = color;
                        }
                        else if (width2 > width && height2 > height)
                        {
                            spindexerBalls[intakeIndex] = color2;
                        }








                        justIndexed = color == 0;
                        ballAmount++;
                    }








                    // --------- QuickShoot

















                    // --------- PID??? ------------
















                    if(gamepad1.a && !prevA)
                    {
                        shootIndex = (shootIndex + 1) % SHOOT_POS.length;
                    }
                    prevA = gamepad1.a;








                    if(gamepad1.b && !prevB)
                    {
                        intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                        shooterSystem.setIntakeIndex(intakeIndex);
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








                    // Manual controls for target and PID tuning
































                    //INTAKAE POS: 0.047, 0.42, 0.81
                    //Shooter POS: 0.235, 0.61, 0.995








                    //Posservo.setPosition(0.995);
















                    // ---------------- AutoShot toggle ----------------
                    boolean startNow = gamepad1.x;
                    if (startNow && !prevStart) {
                        autoShotEnabled = !autoShotEnabled;
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
























                    double ang = shooterSystem.spindexerEncoder.getVoltage();
                    //telemetry.addData("Spindexer Target (rad)", shooterSystem.targetRad);
                    telemetry.addData("Spindexer Angle", Math.toDegrees(ang));










                    double servoDeg = (servoEncoder.getVoltage() / 3.3) * 360.0;








                    telemetry.addData("MelonEnc (deg)", melonDeg);
                    //telemetry.addData("ServoEnc (deg)", servoDeg);
                    telemetry.addData("HoodDeg", shooterSystem.hoodDeg);
                    telemetry.addData("Shooter Target RPM", shooterSystem.getShooterTargetRpm());
                    telemetry.addData("Shooter RPM", shooterSystem.getShooterRpm());
                    telemetry.addData("Hood Target", shooterSystem.getHoodTargetPos());












                    telemetry.addData("Spindexer1", spindexerBalls[0]);
                    telemetry.addData("Spindexer2", spindexerBalls[1]);
                    telemetry.addData("Spindexer3", spindexerBalls[2]);
                    telemetry.addData("intakeIndex", intakeIndex);
                    telemetry.addData("color", color);
                    telemetry.addData("color2", color2);
                    telemetry.addData("blocks length", blocks.length);
                    telemetry.addData("Angle1", Math.abs(spinAngles[0] - melonDeg) < 1);
                    telemetry.addData("Angle2", Math.abs(spinAngles[1] - melonDeg) < 1);
                    telemetry.addData("Angle3", Math.abs(spinAngles[2] - melonDeg) < 1);
                    telemetry.addData("PosServo", Math.abs(spinAngles[3] - melonDeg) < 1);
                    telemetry.addData("ColoredKicker", kickerActive);
                        telemetry.addData("Obelisk Tag", cachedObeliskTag);
                    telemetry.addData("Obelisk Pattern",
                            cachedObeliskPattern == null ? "NONE" : Arrays.toString(cachedObeliskPattern));









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





        }
