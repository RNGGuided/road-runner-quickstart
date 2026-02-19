package org.firstinspires.ftc.teamcode;

// RR-specific imports
import androidx.annotation.RequiresPermission;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

@Autonomous(name = "BLUEBACK9")
public class Blue9 extends LinearOpMode {
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

        int[] spindexerBalls = new int[3];
        ShooterSystem shooter = new ShooterSystem(hardwareMap);
        ShooterActions SA = new ShooterActions(shooter);

        colorMgr = new AutonColorManager();

        colorMgr.onIntake(0, AutonColorManager.BallColor.GREEN);
        colorMgr.onIntake(1, AutonColorManager.BallColor.PURPLE);
        colorMgr.onIntake(2, AutonColorManager.BallColor.PURPLE);
        Action Shootfar= new SequentialAction
                (
                        SA.autonMoveToNextShootIndex(colorMgr),
                        SA.setShooterRpm(4203),
                        SA.waitUntil(() -> shooter.atShooterSpeed(), 2, true),
                        SA.setHoodDeg(0.012),
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
                colorMgr.resetMotifIndex();   // ⭐ ONLY reset order, not contents
            }
        });



        Action Shootfar2= new SequentialAction
                (
                        SA.intakeReverse(.8),
                        SA.autonMoveToNextShootIndex(colorMgr),
                        SA.setShooterRpm(4210),
                        SA.setHoodDeg(.0115),
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
        Action Shootfar1= new SequentialAction
                (
                        SA.intakeReverse(1.0),
                        SA.autonMoveToNextShootIndex(colorMgr),
                        SA.setShooterRpm(4215),
                        SA.setHoodDeg(.012),
                        new SleepAction(.75 ),
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
        //Action FullShot = new SeqeuentialAction()
        Action Intake = new SequentialAction(
                new InstantAction(() -> {
                    colorMgr.clearSpindexer();
                })
                ,
                SA.setIntakeIndex(0),
                SA.intakeForward(.9),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE),
                new SleepAction(.85),
                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.GREEN),
                new SleepAction(.45),
                SA.indexNextIntakeSlot(.6),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE)
        );
        Action Intake2 = new SequentialAction(
                new InstantAction(() -> {
                    colorMgr.clearSpindexer();
                }),
                SA.setIntakeIndex(0),
                SA.intakeForward(.9),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.GREEN),
                new SleepAction(.85),
                SA.indexNextIntakeSlot(.65),
                SA.autonRegisterIntake(colorMgr, AutonColorManager.BallColor.PURPLE),
                new SleepAction(.5),
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
        Pose2d initialPose = new Pose2d(-14.509, -63.381, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize slides

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterTime(0, Shootfar)
                .strafeToLinearHeading(new Vector2d(-14.509, -58.671), Math.toRadians(290.5))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-28.5, -36.4), Math.toRadians(180))
                .afterTime(0, Intake2)
                .strafeTo(new Vector2d(-51, -36.4), new TranslationalVelConstraint(8))
                .afterTime(1.4, Shootfar1)
                .strafeToLinearHeading(new Vector2d(-14.509, -58.671), Math.toRadians(289.4))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-28.25, -10.75), Math.toRadians(180))
                .afterTime(0, Intake)
                .strafeTo(new Vector2d(-51, -10.75), new TranslationalVelConstraint(7.5))
                .afterTime(2.2 , Shootfar2)
                .strafeToLinearHeading(new Vector2d(-14.509, -58.671), Math.toRadians(288.3))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-36.295, -62.596), Math.toRadians(292));

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
                ReadLimelight,
                new ParallelAction(
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