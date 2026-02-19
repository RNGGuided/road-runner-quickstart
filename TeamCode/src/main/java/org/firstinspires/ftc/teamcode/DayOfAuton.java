package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.ParallelAction;

@Autonomous(name = "REDFRONTREAL")
public class DayOfAuton extends LinearOpMode {

    private ShooterSystem shooter;
    private ShooterActions SA;
    private Limelight3A limelight;
    private AutonColorManager colorMgr;

    @Override
    public void runOpMode() throws InterruptedException {

        // ==================== INIT ====================
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        shooter = new ShooterSystem(hardwareMap);
        SA = new ShooterActions(shooter);
        colorMgr = new AutonColorManager();

        // preload (same as BlueFront)
        colorMgr.onIntake(0, AutonColorManager.BallColor.GREEN);
        colorMgr.onIntake(1, AutonColorManager.BallColor.PURPLE);
        colorMgr.onIntake(2, AutonColorManager.BallColor.PURPLE);

        // ==================== ACTIONS (IDENTICAL TO BLUEFRONT) ====================

        Action S = new SequentialAction(
                SA.setShooterRpm(3400),
                SA.waitUntil(() -> shooter.atShooterSpeed(), 1.6, true),
                SA.autonMoveToNextShootIndex(colorMgr),
                SA.setHoodDeg(.485),
                new SleepAction(0.3),
                SA.kickerUp(),
                new SleepAction(0.4),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),

                new SleepAction(0.1),
                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.45),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),

                new SleepAction(0.25),
                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.4),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr)
        );

        Action S2 = new SequentialAction(
                SA.intakeReverse(1.0),

                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(0.4),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),

                new SleepAction(0.2),

                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.45),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),

                new SleepAction(0.3),

                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.4),
                SA.kickerUp(),
                new SleepAction(0.25),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr)
        );

        Action S3 = new SequentialAction(
                SA.intakeReverse(1.0),
                SA.setHoodDeg(.485),

                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(0.4),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),

                new SleepAction(0.2),

                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.45),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr),

                new SleepAction(0.3),

                SA.autonMoveToNextShootIndex(colorMgr),
                new SleepAction(.4),
                SA.kickerUp(),
                new SleepAction(0.2),
                SA.kickerDown(),
                SA.autonConfirmShot(colorMgr)
        );

        Action Intake = new SequentialAction(
                new InstantAction(() -> colorMgr.clearSpindexer()),
                SA.setIntakeIndex(0),
                SA.intakeForward(.9),

                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE),
                new SleepAction(1.2),

                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE),
                new SleepAction(.53),

                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.GREEN)
        );

        Action Intake2 = new SequentialAction(
                new InstantAction(() -> colorMgr.clearSpindexer()),
                SA.setIntakeIndex(0),
                SA.intakeForward(1.0),

                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE),
                new SleepAction(1.22),

                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.GREEN),
                new SleepAction(.5 ),

                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE)
        );

        Action ReadLimelightFor1s = new Action() {
            private double startTime = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (startTime < 0) startTime = System.nanoTime() / 1e9;

                double now = System.nanoTime() / 1e9;

                AutonColorManager.BallColor[] motif =
                        SA.readMotifFromLimelight(limelight);

                if (motif != null) {
                    colorMgr.setMotif(motif);
                    colorMgr.resetMotifIndex();
                }

                return (now - startTime) < 5.0;
            }
        };

        // ==================== MIRRORED INITIAL POSE ====================
        Pose2d initialPose = new Pose2d(50.231, 49.674, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // ==================== MIRRORED TRAJECTORY ====================
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterTime(2.3, S)

                .strafeToLinearHeading(new Vector2d(14.543, 12), Math.toRadians(290))
                .turnTo(Math.toRadians(224))
                .waitSeconds(3)

                .strafeToLinearHeading(new Vector2d(27, 9.25), Math.toRadians(2))
                .afterTime(0, Intake)

                .strafeTo(new Vector2d(50, 9.25), new TranslationalVelConstraint(8))
                .afterTime(1.55, S2)

                .strafeToLinearHeading(new Vector2d(13.527, 17), Math.toRadians(222.5))
                .waitSeconds(2.5)

                .strafeToLinearHeading(new Vector2d(27, -14), Math.toRadians(2))
                .afterTime(0, Intake2)

                .strafeTo(new Vector2d(51, -14), new TranslationalVelConstraint(8))
                .afterTime(2, S3)

                .strafeToLinearHeading(new Vector2d(10, 17), Math.toRadians(222))
                .waitSeconds(3.5)

                .strafeTo(new Vector2d(32, -3.517));

        // ==================== PRESTART ====================
        Actions.runBlocking(
                new ParallelAction(
                        SA.huskyUp(),
                        SA.keepUpdatingFor(2),
                        SA.setIntakeIndex(0)
                )
        );

        waitForStart();

        // ==================== AUTON ====================
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                ReadLimelightFor1s,
                                SA.keepUpdatingFor(30),
                                new SequentialAction(tab1.build())
                        )
                )
        );
    }
}
