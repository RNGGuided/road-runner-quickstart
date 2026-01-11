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

@Autonomous(name = "REDBACK6")
public class RedBack6 extends LinearOpMode {
    private ShooterSystem shooter;

    private ShooterActions SA;

    @Override
    public void runOpMode() throws InterruptedException {
        int[] spindexerBalls = new int[3];
        ShooterSystem shooter = new ShooterSystem(hardwareMap);
        ShooterActions SA = new ShooterActions(shooter, spindexerBalls);

        Action Shootfar= new SequentialAction
                (
                        SA.setShooterRpm(4175),
                        SA.waitUntil(() -> shooter.atShooterSpeed(), 5, true),
                        SA.setHoodDeg(220),
                        new SleepAction(1 ),
                        SA.kickerUp(),
                        new SleepAction(0.2),
                        SA.kickerDown(),
                        new SleepAction(0.2),
                        SA.indexNextShootSlot(1),
                        new SleepAction(1.5 ),
                        SA.kickerUp(),
                        new SleepAction(0.2),
                        SA.kickerDown(),
                        new SleepAction(0.25),
                        SA.indexNextShootSlot(1),
                        new SleepAction(1.5),
                        SA.kickerUp(),
                        new SleepAction(0.2),
                        SA.kickerDown()
                );
        Action Shootfar1= new SequentialAction
                (
                        SA.setShooterRpm(4250),
                        SA.setHoodDeg(220),
                        new SleepAction(1 ),
                        SA.kickerUp(),
                        new SleepAction(0.2),
                        SA.kickerDown(),
                        new SleepAction(0.2),
                        SA.indexNextShootSlot(1),
                        new SleepAction(1.5 ),
                        SA.kickerUp(),
                        new SleepAction(0.2),
                        SA.kickerDown(),
                        new SleepAction(0.25),
                        SA.indexNextShootSlot(1),
                        new SleepAction(1.5),
                        SA.kickerUp(),
                        new SleepAction(0.2),
                        SA.kickerDown(),
                        new SleepAction(0.25),
                        SA.indexNextShootSlot(1),
                        new SleepAction(1.5),
                        SA.kickerUp(),
                        new SleepAction(0.2),
                        SA.kickerDown()
                );
        Action S2= new SequentialAction
                (
                        SA.intakeReverse(1.0),
                        SA.indexNextShootSlot(1),
                        new SleepAction(0.4),
                        SA.kickerUp(),
                        new SleepAction(0.2),
                        SA.kickerDown(),
                        new SleepAction(0.2),
                        SA.indexNextShootSlot(1),
                        new SleepAction(.45),
                        SA.kickerUp(),
                        new SleepAction(0.2),
                        SA.kickerDown(),
                        new SleepAction(0.3),
                        SA.indexNextShootSlot(1),
                        new SleepAction(.4),
                        SA.kickerUp(),
                        new SleepAction(0.2),
                        SA.kickerDown()

                );
        Action S3= new SequentialAction
                (
                        SA.intakeReverse(1.0),
                        SA.setHoodDeg(167),
                        SA.indexNextShootSlot(1),
                        new SleepAction(0.4),
                        SA.kickerUp(),
                        new SleepAction(0.2),
                        SA.kickerDown(),
                        new SleepAction(0.2),
                        SA.indexNextShootSlot(1),
                        new SleepAction(.45),
                        SA.kickerUp(),
                        new SleepAction(0.2),
                        SA.kickerDown(),
                        new SleepAction(0.3),
                        SA.indexNextShootSlot(1),
                        new SleepAction(.4),
                        SA.kickerUp(),
                        new SleepAction(0.2),
                        SA.kickerDown()

                );
        //Action FullShot = new SeqeuentialAction()
        Action Intake = new SequentialAction(
                SA.intakeForward(.9),
                SA.indexNextIntakeSlot(.3),
                new SleepAction(.67),
                SA.indexNextIntakeSlot(.35),
                new SleepAction(.22 ),
                SA.indexNextIntakeSlot(.35)
                /*new SleepAction(.2),
                SA.indexNextIntakeSlot(.35)*/
        );
        Action Intake2 = new SequentialAction(
                SA.intakeForward(1.0),
                SA.indexNextIntakeSlot(.3),
                new SleepAction(.85),
                SA.indexNextIntakeSlot(.35),
                new SleepAction(.2 ),
                SA.indexNextIntakeSlot(.35)
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
        Pose2d initialPose = new Pose2d(14.509, -63.381, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize slides

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterTime(0,Shootfar)
                .strafeToLinearHeading(new Vector2d(14.509, -58.671), Math.toRadians(247))
                .waitSeconds(10)
                .strafeToLinearHeading(new Vector2d(29, -37.277), Math.toRadians(0))
                .afterTime(0,Intake2)
                .strafeTo(new Vector2d(51, -37.277), new TranslationalVelConstraint(9 ))
                .afterTime(2,Shootfar1)
                .strafeToLinearHeading(new Vector2d(14.509, -58.671), Math.toRadians(247))
                .waitSeconds(10)
                .strafeToLinearHeading(new Vector2d(36.295, -62.596), Math.toRadians(247));

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

                        SA.keepUpdatingFor(5),
                        SA.indexNextShootSlot(3.0)
                )
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