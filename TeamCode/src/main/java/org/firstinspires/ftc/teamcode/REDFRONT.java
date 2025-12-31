package org.firstinspires.ftc.teamcode;

// RR-specific imports
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import java.util.Arrays;

@Autonomous(name = "RedFront")
public class REDFRONT extends LinearOpMode {
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
                        //SA.setShooterMode(ShooterSystem.ShooterControlMode.HYBRID),
                        //SA.spinUpRpm(1675),
                        //SA.waitUntil(() -> shooter.atShooterSpeed(), 2.5, false),
                        new SleepAction(0.2),
                        SA.kickerDown(),
                        new SleepAction(0.2),
                        SA.kickerUp(),
                        SA.indexNextAngle(1.5),
                        new SleepAction(.5),
                        SA.kickerDown(),
                        new SleepAction(0.3),
                        SA.kickerUp(),
                        SA.indexNextAngle(1.5),
                        new SleepAction(.5),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp()

                );
        Action S2= new SequentialAction
                (
                        //SA.feedFull(.1),
                        //SA.indexNextAngle(1.5),
                        SA.intakeReverse(1.0),
                        SA.indexNextAngle(1.5),
                        //SA.setShooterMode(ShooterSystem.ShooterControlMode.HYBRID),
                        //SA.spinUpRpm(1675),
                        //SA.waitUntil(() -> shooter.atShooterSpeed(), 1.0, true),
                        new SleepAction(0.2),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp(),
                        SA.indexNextAngle(1.5),//
                        new SleepAction(.4),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp(),
                        SA.indexNextAngle(1.5),
                        new SleepAction(.4),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp()

                );
        Action S3= new SequentialAction
                (
                        //SA.feedFull(.1),
                        //SA.indexNextAngle(1.5),
                        //SA.setShooterMode(ShooterSystem.ShooterControlMode.HYBRID),
                        SA.intakeReverse(1.0),
                        SA.indexNextAngle(1.5),
                        SA.spinUpRpm(1675),
                        SA.waitUntil(() -> shooter.atShooterSpeed(), 1.0, true),
                        new SleepAction(0.2),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp(),
                        SA.indexNextAngle(1.5),//
                        new SleepAction(.4),
                        SA.kickerDown(),
                        new SleepAction(0.5),
                        SA.kickerUp(),
                        SA.indexNextAngle(1.5),
                        new SleepAction(.4),
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
                SA.indexNextAngle(1.3),
                SA.intakeForward(1.0),
                new SleepAction(1),
                SA.indexNextAngle(1.3),
                SA.indexNextAngle(1.3),
                new SleepAction(.05),
                SA.indexNextAngle(1.5)
        );
        Action Intake2 = new SequentialAction(
                SA.indexNextAngle(1.3),
                SA.intakeForward(1.0),
                new SleepAction(.367425),
                SA.indexNextAngle(1.3),
                SA.indexNextAngle(1.3),
                new SleepAction(.05),
                SA.indexNextAngle(1.5)
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
        Pose2d initialPose = new Pose2d(-51.605, 50.459, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize slides

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterTime(0,S)
                .strafeToLinearHeading(new Vector2d(-14.543,13.355),Math.toRadians(135))
                .waitSeconds(4.5)
                .turnTo((Math.toRadians(180)))
                .afterTime(0,Intake)
                .strafeTo(new Vector2d(-50, 11.288), new TranslationalVelConstraint(15 ))
                .afterTime(1,S2)
                .strafeToLinearHeading(new Vector2d(-10,17 ),Math.toRadians(135))
                .waitSeconds(4.5)
                .strafeToLinearHeading(new Vector2d(-26.942,-10),Math.toRadians(180))
                .afterTime(0,Intake2)
                .strafeTo(new Vector2d(-51, -10), new TranslationalVelConstraint(15 ))
                .afterTime(.75,S3)
                .strafeToLinearHeading(new Vector2d(10,17),Math.toRadians(135))
                .waitSeconds(4)
                .strafeTo(new Vector2d(-25.108, -3.517));

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
                SA.indexNextAngle(3.0)

        );
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