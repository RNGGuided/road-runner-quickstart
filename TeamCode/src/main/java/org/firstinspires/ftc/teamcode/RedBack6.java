package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;

@Autonomous(name = "REDFIN")
public class RedBack6 extends LinearOpMode {

    private ShooterSystem shooter;
    private ShooterActions SA;
    private Limelight3A limelight;
    private AutonColorManager colorMgr;
    private HuskyLens husky;
    boolean stopIndexing = false;
    @Override
    public void runOpMode() throws InterruptedException {

        husky = hardwareMap.get(HuskyLens.class, "husky");
        husky.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");
        limelight.pipelineSwitch(0);
        limelight.start();

        colorMgr = new AutonColorManager();

        ShooterSystem shooter = new ShooterSystem(hardwareMap);
        ShooterActions SA = new ShooterActions(shooter);

        colorMgr.onIntake(0, AutonColorManager.BallColor.GREEN);
        colorMgr.onIntake(1, AutonColorManager.BallColor.PURPLE);
        colorMgr.onIntake(2, AutonColorManager.BallColor.PURPLE);

        // ---------------- EXACT BLUE9 ACTIONS ----------------
        Action ReadLimelightFor1s = new Action() {

            private double startTime = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // initialize timer once
                if (startTime < 0) {
                    startTime = System.nanoTime() / 1e9;
                }

                double now = System.nanoTime() / 1e9;

                // ⭐ keep reading every loop for 1 second
                AutonColorManager.BallColor[] motif =
                        SA.readMotifFromLimelight(limelight);

                if (motif != null) {
                    colorMgr.setMotif(motif);
                    colorMgr.resetMotifIndex();
                }

                // return true while still running
                return (now - startTime) < 2.0;
            }
        };
        Action Shootfar = new SequentialAction(
                SA.autonMoveToNextShootIndex(colorMgr),
                SA.setShooterRpm(4300),
                SA.waitUntil(() -> shooter.atShooterSpeed(), 2, true),
                SA.setHoodDeg(0.025),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.3),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),
                new SleepAction(0.3),

                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.1),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),

                new SleepAction(0.35),
                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr)
        );
        Action detectAndIndexBalls = new Action() {

            boolean readyToIndex = true;
            long lastIndexTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (stopIndexing) {
                    return false; // stop the action immediately
                }

                HuskyLens.Block[] blocks = husky.blocks();

                boolean ballDetected = false;
                int detectedColor = -1;

                if (blocks.length > 0) {

                    HuskyLens.Block b = blocks[0];

                    if (b.id == 1) { // purple
                        detectedColor = 1;
                        ballDetected = true;
                    }

                    if (b.id == 2) { // green
                        detectedColor = 2;
                        ballDetected = true;
                    }
                }

                if (ballDetected && readyToIndex &&
                        System.currentTimeMillis() - lastIndexTime > 300) {

                    AutonColorManager.BallColor color =
                            detectedColor == 1
                                    ? AutonColorManager.BallColor.PURPLE
                                    : AutonColorManager.BallColor.GREEN;

                    colorMgr.onIntake(shooter.intakeIndex, color);

                    shooter.stepIntakeSlot();

                    lastIndexTime = System.currentTimeMillis();
                    readyToIndex = false;
                }

                if (!ballDetected) {
                    readyToIndex = true;
                }

                return true;
            }
        };
        Action detectAndIndexBalls2 = new Action() {

            boolean readyToIndex = true;
            long lastIndexTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (stopIndexing) {
                    return false; // stop the action immediately
                }

                HuskyLens.Block[] blocks = husky.blocks();

                boolean ballDetected = false;
                int detectedColor = -1;

                if (blocks.length > 0) {

                    HuskyLens.Block b = blocks[0];

                    if (b.id == 1) { // purple
                        detectedColor = 1;
                        ballDetected = true;
                    }

                    if (b.id == 2) { // green
                        detectedColor = 2;
                        ballDetected = true;
                    }
                }

                if (ballDetected && readyToIndex &&
                        System.currentTimeMillis() - lastIndexTime > 300) {

                    AutonColorManager.BallColor color =
                            detectedColor == 1
                                    ? AutonColorManager.BallColor.PURPLE
                                    : AutonColorManager.BallColor.GREEN;

                    colorMgr.onIntake(shooter.intakeIndex, color);

                    shooter.stepIntakeSlot();

                    lastIndexTime = System.currentTimeMillis();
                    readyToIndex = false;
                }

                if (!ballDetected) {
                    readyToIndex = true;
                }

                return true;
            }
        };
        Action autoIndexing = new SequentialAction(
                SA.setIntakeIndex(0),
                SA.intakeForward(.85),
                new InstantAction(() -> {
                    SA.runAutoIndexingFor(4.5);
                }));

        Action ReadLimelight = new InstantAction(() -> {
            AutonColorManager.BallColor[] motif =
                    SA.readMotifFromLimelight(limelight);
            if (motif != null) {
                colorMgr.setMotif(motif);
                colorMgr.resetMotifIndex();
            }
        });

        Action Shootfar2 = new SequentialAction(
                SA.intakeReverse(.8),
                SA.indexNextShootSlot(1),
                SA.setShooterRpm(4300),
                SA.setHoodDeg(0.02),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                new SleepAction(0.2),

                SA.indexNextShootSlot(1),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.1),
                SA.kickerDown(),

                new SleepAction(0.3),
                SA.indexNextShootSlot(1),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown()
        );

        Action Shootfar1 = new SequentialAction(
                SA.intakeReverse(1.0),
                SA.indexNextShootSlot(1),
                SA.setShooterRpm(4300),
                SA.setHoodDeg(0.02),
                new SleepAction(.75),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                new SleepAction(0.25),

                SA.indexNextShootSlot(1),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),

                new SleepAction(0.35),
                SA.indexNextShootSlot(1),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown()
                ,SA.intakeForward(.83)
        );

        Action Intake = new ParallelAction(
                detectAndIndexBalls,
                new SequentialAction(
                        new InstantAction(() -> colorMgr.clearSpindexer()),
                        SA.setIntakeIndex(0),
                        SA.intakeForward(.7 ),
                        new SleepAction(2.0) // intake time
                )
        );

        Action Intake2 = new ParallelAction(
                detectAndIndexBalls2,
                new SequentialAction(
                        new InstantAction(() -> colorMgr.clearSpindexer()),
                        SA.setIntakeIndex(0),
                        SA.intakeForward(.7),
                        new SleepAction(2.0)
                )
        );
        Action IntakeSpec = new SequentialAction(
                new InstantAction(() -> colorMgr.clearSpindexer()),
                SA.setIntakeIndex(0),
                SA.intakeForward(1),
                new SleepAction(.1),
                SA.indexNextIntakeSlot(.6),
                new SleepAction(.05),
                SA.indexNextIntakeSlot(.6),
                new SleepAction(.05),
                SA.indexNextIntakeSlot(.6),
                new SleepAction(.05),
                SA.indexNextIntakeSlot(.6)
        );

        // ---------------- YOUR RED PATH (UNCHANGED) ----------------

        Pose2d initialPose = new Pose2d(14.509, -63.381, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterTime(0, Shootfar)
                .strafeToLinearHeading(new Vector2d(14.509, -58.671), Math.toRadians(-111.5))
                .waitSeconds(4.3)
                .afterTime(.5, Intake)
                .strafeToLinearHeading(new Vector2d(62.824, -63), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(52, -63), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(62.824, -64.167), Math.toRadians(10))
                .waitSeconds(1)
                .afterTime(3, new SequentialAction(
                        new InstantAction(() -> stopIndexing = true),
                        Shootfar1
                ))
                .strafeToLinearHeading(new Vector2d(14.509, -58.671), Math.toRadians(-111.5))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(36.295, -62.596), Math.toRadians(252));

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(.5),
                        new ParallelAction(
                                ReadLimelightFor1s,
                                SA.keepUpdatingFor(30),
                                new SequentialAction(
                                        tab1.build()
                                )
                        )
                )
        );
    }
}
