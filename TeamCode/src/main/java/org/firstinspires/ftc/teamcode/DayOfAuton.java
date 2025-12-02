package org.firstinspires.ftc.teamcode;

// RR-specific imports
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

@Autonomous(name = "BlueFront")
public class DayOfAuton extends LinearOpMode {
    private ShooterSystem shooter;

    private ShooterActions SA;

    @Override
    public void runOpMode() throws InterruptedException {
        int[] spindexerBalls = new int[3];
        ShooterSystem shooter = new ShooterSystem(hardwareMap);
        ShooterActions SA = new ShooterActions(shooter, spindexerBalls);

        Action S= new SequentialAction
                (
                        SA.feedFull(.25),
                        SA.setShooterMode(ShooterSystem.ShooterControlMode.HYBRID),
                SA.spinUpRpm(1685),
                        SA.waitUntil(() -> shooter.atShooterSpeed(), 3.0, true),
                        new SleepAction(0.2),
                SA.kickerDown(),
                        new SleepAction(0.5),
                SA.kickerUp(),
                SA.indexNextAngle(1.5),
                        new SleepAction(.2),
                SA.kickerDown(),
                        new SleepAction(0.5),
                SA.kickerUp(),
                SA.indexNextAngle(1.5),
                        new SleepAction(.5),
                SA.kickerDown(),
                        new SleepAction(0.5),
                SA.kickerUp(),
                        SA.indexNextAngle(1.5),
                        new SleepAction(.5  ),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp()

        );
        Action S2= new SequentialAction
                (
                        SA.feedFull(.1),
                        //SA.indexNextAngle(1.5),
                        SA.setShooterMode(ShooterSystem.ShooterControlMode.HYBRID),
                        SA.spinUpRpm(1700),
                        SA.waitUntil(() -> shooter.atShooterSpeed(), 3.0, true),
                        new SleepAction(0.3),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp(),
                        SA.indexNextAngle(1.5),//
                        new SleepAction(.3),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp(),
                        SA.indexNextAngle(1.5),
                        new SleepAction(.3),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp(),
                        SA.indexNextAngle(1.5),
                        new SleepAction(.7),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp()

                );
        Action S3= new SequentialAction
                (
                        SA.feedFull(.1),
                        //SA.indexNextAngle(1.5),
                        SA.setShooterMode(ShooterSystem.ShooterControlMode.HYBRID),
                        SA.spinUpRpm(1700),
                        SA.waitUntil(() -> shooter.atShooterSpeed(), 3.0, true),
                        new SleepAction(0.3),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp(),
                        SA.indexNextAngle(1.5),//
                        new SleepAction(.3),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp(),
                        SA.indexNextAngle(1.5),
                        new SleepAction(.3),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp(),
                        SA.indexNextAngle(1.5),
                        new SleepAction(.7),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp()

                );
        Action Second = new SequentialAction (
                SA.indexNextAngle(1.5),
                new SleepAction(1.0),
                SA.kickerDown(),
                new SleepAction(0.5),
                SA.kickerUp()
        );
        //Action FullShot = new SeqeuentialAction()
        Action Intake = new SequentialAction(
                SA.indexNextAngle(1),
                SA.intakeForward(1.0),
                new SleepAction(1),
                SA.indexNextAngle(1),
                SA.indexNextAngle(1),
                new SleepAction(.05),
                SA.indexNextAngle(1)
        );
        Action Intake2 = new SequentialAction(
                SA.indexNextAngle(1),
                SA.intakeForward(1.0),
                new SleepAction(1),
                SA.indexNextAngle(1),
                SA.indexNextAngle(1),
                new SleepAction(.05),
                SA.indexNextAngle(1)
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
        Pose2d initialPose = new Pose2d(15.06, 63.984, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize slides

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterTime(0,S)
                .strafeToLinearHeading(new Vector2d(14.543,13.355),Math.toRadians(45))
                .waitSeconds(8)
                .turnTo((Math.toRadians(0)))
                .afterTime(0,Intake)
                .strafeTo(new Vector2d(53.807, 11.288), new TranslationalVelConstraint(15 ))
                .afterTime(0,S2)
                .strafeToLinearHeading(new Vector2d(10,13),Math.toRadians(45))
                .waitSeconds(8)
                .strafeToLinearHeading(new Vector2d(26.942,-13.251),Math.toRadians(0))
                .afterTime(0,Intake2)
                .strafeTo(new Vector2d(54.323, -13.51), new TranslationalVelConstraint(15 ))
                .afterTime(0,S3)
                .strafeToLinearHeading(new Vector2d(14.543,13.355),Math.toRadians(43));
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

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        SA.keepUpdatingFor(30),
                new SequentialAction(
                        // SA.indexNextAngle(1.5),
                        tab1.build()
                )
            )
        );

    }
}