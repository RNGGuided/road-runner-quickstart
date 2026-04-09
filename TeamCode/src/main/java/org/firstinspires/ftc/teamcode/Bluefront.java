package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import java.util.Arrays;

@Autonomous(name = "BlueFront")
public class Bluefront extends LinearOpMode {
    private ShooterSystem shooter;

    private ShooterActions SA;
    private Limelight3A limelight;
    private AutonColorManager colorMgr;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");
        limelight.pipelineSwitch(0);
        limelight.start();

        colorMgr = new AutonColorManager();

        int[] spindexerBalls = new int[3];
        ShooterSystem shooter = new ShooterSystem(hardwareMap);
        ShooterActions SA = new ShooterActions(shooter);

        colorMgr = new AutonColorManager();

        colorMgr.onIntake(0, AutonColorManager.BallColor.GREEN);
        colorMgr.onIntake(1, AutonColorManager.BallColor.PURPLE);
        colorMgr.onIntake(2, AutonColorManager.BallColor.PURPLE);
        Action S= new SequentialAction
                (
                        SA.setShooterRpm(3400),
                        SA.waitUntil(() -> shooter.atShooterSpeed(), 1.6, true),
                        SA.autonMoveToNextShootIndex(colorMgr),
                        SA.setHoodDeg(.46),
                        new SleepAction(0.3 ),
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

        //Action FullShot = new SeqeuentialAction()
        Action Intake = new SequentialAction(
                new InstantAction(() -> colorMgr.clearSpindexer()),
                SA.setIntakeIndex(0),
                SA.intakeForward(.9),

                SA.autonRegisterIntake( colorMgr, AutonColorManager.BallColor.PURPLE),
                new SleepAction(1.02),

                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE),
                new SleepAction(.53),

                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.GREEN)
        );
        Action ReadLimelight = new InstantAction(() -> {
            AutonColorManager.BallColor[] motif =
                    SA.readMotifFromLimelight(limelight);
            if (motif != null) {
                colorMgr.setMotif(motif);
                colorMgr.resetMotifIndex();   // ⭐ ONLY reset order, not contents
            }
        });
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
                return (now - startTime) < 5.0;
            }
        };



        Action Intake2 = new SequentialAction(
                new InstantAction(() -> colorMgr.clearSpindexer()),
                SA.setIntakeIndex(0),
                SA.intakeForward(1.0),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE),
                new SleepAction(1.02),

                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.GREEN),
                new SleepAction(.45),

                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE)
        );


        // myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9, 60, Math.toRadians(90)))
        // .strafeTo(new Vector2d(37, 63))
        // .strafeTo(new Vector2d(30, 63))
        // .strafeTo(new Vector2d(24, 30))
        // .turn((Math.toRadians(140)))
        // .strafeTo(new Vector2d(17.8, 14.5))
        // .turn((Math.toRadians(-135)))
        // .strafeTo(new Vector2d(12.5, 34))

        //Pose2d initialPose = new Pose2d(9, 61.5, Math.toRadians(270));
        Pose2d initialPose = new Pose2d(-50.231, 49.674, Math.toRadians(315));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize slides

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterTime(2,S)
                .strafeToLinearHeading(new Vector2d(-14.543,12),Math.toRadians(270))
                .turnTo(Math.toRadians(316))
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-27,9.25),Math.toRadians(178))
                .afterTime(0,Intake)
                .strafeTo(new Vector2d(-50, 9.25 ), new TranslationalVelConstraint(8))
                .afterTime(1.55,S2)
                .strafeToLinearHeading(new Vector2d(-13.527  ,17 ),Math.toRadians(317.5))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(-27,-14),Math.toRadians(178))
                .afterTime(0,Intake2)
                .strafeTo(new Vector2d(-51, -14), new TranslationalVelConstraint(8 ))
                .afterTime(2,S3)
                .strafeToLinearHeading(new Vector2d(-10,17),Math.toRadians(318))
                .waitSeconds(3.5)
                .strafeTo(new Vector2d(-32, -3.517));

                /*.strafeTo(new Vector2d(-6,45))
                .splineToLinearHeading(new Pose2d(-35,36,Math.toRadians(180)), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-38, 12), Math.PI)
                .splineToConstantHeading(new Vector2d(-38, 12), Math.PI)
                .strafeTo(new Vector2d(-48, 12))
                .strafeTo(new Vector2d(-48, 60), new TranslationalVelConstraint(40.0), new ProfileAccelConstraint(-10.0, 30))
                .strafeTo(new Vector2d(-48, 12))
                .splineToLinearHeading(new Pose2d(-57,12,Math.toRadians(180)), Math.toRadians(180))
                .strafeTo(new Vector2d(-57, 12))
                .strafeTo(new Vector2d(-57, 55))
                .strafeTo(new Vector2d(-57,50))
                .strafeToLinearHeading(new Vector2d(-29,57), Math.PI)
                .waitSeconds(3)
                .splineToLinearHeading(new Pose2d(-8,34,3*Math.PI/2),3*Math.PI/2)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-33,57), Math.PI)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-8,34,3*Math.PI/2),3*Math.PI/2)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-33,57), Math.PI)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-8,50), 3*Math.PI/2)
                .strafeTo(new Vector2d(-8, 35))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-35, 50));*/

        // .strafeTo(new Vector2d(-36, 60))
        // .strafeTo(new Vector2d(-36, 12))
        // .strafeTo(new Vector2d(-48, 12))
        // .strafeTo(new Vector2d(-48, 60))
        // .strafeTo(new Vector2d(-48, 12))
        // .strafeTo(new Vector2d(-57, 12))
        // .strafeTo(new Vector2d(-57, 60))
        // .strafeTo(new Vector2d(-57, 12))
        // .strafeTo(new Vector2d(-60, 12))
        // .strafeTo(new Vector2d(-60, 60));

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

                        new ParallelAction(
                                ReadLimelightFor1s,
                                SA.keepUpdatingFor(30),
                                new SequentialAction(
                                        // SA.indexNextAngle(1.5),
                                        tab1.build()
                                )
                        )
                )
        );

    }
}