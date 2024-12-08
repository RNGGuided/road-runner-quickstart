package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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

@Autonomous(name = "LeftSide Auton 1st comp")
public class MainAutoMode extends LinearOpMode {

    private Slides slides; // Declare slides
    private Arm arm;
    private Claw claw;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap);
        slides = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);

        Action Pick = new SequentialAction(
                new SleepAction(.5),
                arm.moveToPositionActionArm(1,1),
                new SleepAction(2),
                slides.moveSlidesToHeightAction(17, .75),
                new SleepAction(.5),
                claw.setPositionActionClaw(1),
                new SleepAction(.4),
                arm.moveToPositionActionArm(.5,.5),
                slides.moveSlidesToHeightAction(49, 1),
                new SleepAction(1.4),
                claw.setPositionActionClaw(0)
        );
        Action Pick2 = new SequentialAction(
                new ParallelAction(
                slides.moveSlidesToHeightAction(17, .75),
                arm.moveToPositionActionArm(1,1)),
                new SleepAction(.7),
                slides.moveSlidesToHeightAction(17, .75),
                claw.setPositionActionClaw(1),
                new SleepAction(1),
                arm.moveToPositionActionArm(.5,.5),
                slides.moveSlidesToHeightAction(49, 1),
                new SleepAction(3),
                claw.setPositionActionClaw(0),
                new SleepAction(1),
                arm.moveToPositionActionArm(0,0)
        );
        Action Pick3 = new SequentialAction(
                slides.moveSlidesToHeightAction(17, .75),
                arm.moveToPositionActionArm(1,1),
                new SleepAction(.7),
                slides.moveSlidesToHeightAction(17, .75),
                claw.setPositionActionClaw(1),
                new SleepAction(1),
                arm.moveToPositionActionArm(.5,.5),
                slides.moveSlidesToHeightAction(49, 1),
                new SleepAction(3),
                claw.setPositionActionClaw(0),
                new SleepAction(1),
                arm.moveToPositionActionArm(0,0));

        Action Block1Alt = new SequentialAction( new ParallelAction (
                arm.moveToPositionActionArm(.5,.5),
                slides.moveSlidesToHeightAction(27.5,1)),
                new SleepAction(.5),
                slides.moveSlidesToHeightAction(22.5,1),
                new SleepAction(.2),
                claw.setPositionActionClaw(0));
        Action Everything = new SequentialAction(
                Block1Alt,
                new SleepAction(1),
                Pick,
                new SleepAction(2.5),
                Pick2,
                new SleepAction(2),
                Pick3

        );
        Action slideTask = slides.moveSlidesToHeightAction(17, 1 );
        Action slideTask1 = slides.moveSlidesToHeightAction(22, .75 );
        Action slideTask2 = slides.moveSlidesToHeightAction(27.5, 1 );
        Action slideTask3 = slides.moveSlidesToHeightAction(45, 1 );
        Action ArmTask1 = arm.moveToPositionActionArm(0,0);
        Action ArmTask2 = arm.moveToPositionActionArm(.5,.5);
        Action ArmTask3 = arm.moveToPositionActionArm(1,1);
        Action ClawClose = claw.setPositionActionClaw(1);

        Pose2d initialPose = new Pose2d(8.5, 61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(6, 35))// (37, 63) -> (6, 32)
                .waitSeconds(1)
                .strafeTo(new Vector2d(6, 39))// (30, 63) -> (6, 39)
                .strafeTo(new Vector2d(48, 44))
                .waitSeconds(4.3)
                .strafeToLinearHeading(new Vector2d(51, 56), 5*Math.PI/18)
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(56.85, 44),  Math.toRadians(265))
                .waitSeconds(5)
                .strafeTo(new Vector2d(53, 44))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(51, 56), 5*Math.PI/18)
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(56.85, 44),  Math.toRadians(300))
                .waitSeconds(4.5)
                .strafeToLinearHeading(new Vector2d(51, 56), 5*Math.PI/18)
                .waitSeconds(2)
                .strafeTo(new Vector2d(30, 10));


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        ClawClose,
                        new ParallelAction(
                                tab1.build(),
                                Everything
                                )
                        )
        );



    }
}
