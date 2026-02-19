package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;

@Autonomous(name = "REDBACK3")
public class RedBack6 extends LinearOpMode {

    private ShooterSystem shooter;
    private ShooterActions SA;
    private Limelight3A limelight;
    private AutonColorManager colorMgr;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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
                SA.setShooterRpm(4207),
                SA.waitUntil(() -> shooter.atShooterSpeed(), 2, true),
                SA.setHoodDeg(0.0125),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.4),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),
                new SleepAction(0.2),

                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),

                new SleepAction(0.25),
                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr)
        );

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
                SA.autonMoveToNextShootIndex(colorMgr),
                SA.setShooterRpm(4210),
                SA.setHoodDeg(.0122),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),
                new SleepAction(0.2),

                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),

                new SleepAction(0.25),
                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr)
        );

        Action Shootfar1 = new SequentialAction(
                SA.intakeReverse(1.0),
                SA.autonMoveToNextShootIndex(colorMgr),
                SA.setShooterRpm(4210),
                SA.setHoodDeg(.0122),
                new SleepAction(.75),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),
                new SleepAction(0.25),

                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),

                new SleepAction(0.25),
                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.5),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr)
        );

        Action Intake = new SequentialAction(
                new InstantAction(() -> colorMgr.clearSpindexer()),
                SA.setIntakeIndex(0),
                SA.intakeForward(.85),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE),
                new SleepAction(1),
                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.GREEN),
                new SleepAction(.54),
                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE)
        );

        Action Intake2 = new SequentialAction(
                new InstantAction(() -> colorMgr.clearSpindexer()),
                SA.setIntakeIndex(0),
                SA.intakeForward(.9),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.GREEN),
                new SleepAction(1),
                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE),
                new SleepAction(.5),
                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE)
        );

        // ---------------- YOUR RED PATH (UNCHANGED) ----------------

        Pose2d initialPose = new Pose2d(14.509, -63.381, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterTime(0, Shootfar)
                .strafeToLinearHeading(new Vector2d(14.509, -58.671),Math.toRadians(245.8))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(36.295, -62.596), Math.toRadians(252));

        Actions.runBlocking(
                new ParallelAction(
                        SA.huskyUp(),
                        SA.keepUpdatingFor(2),
                        SA.setIntakeIndex(0)
                )
        );

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
