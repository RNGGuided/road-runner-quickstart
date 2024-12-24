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

@Autonomous(name = "LeftSide Auton Pray")
public class MainAutoMode1 extends LinearOpMode {

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
                new SleepAction(1.2),//Changed time from 2 to 1.5 11/27/2024, //Changed time from 1.5 to 1.2 11/29/2024
                slides.moveSlidesToHeightAction(17, .75),
                new SleepAction(.3),//Changed time from .5 to .3 11/29/2024
                claw.setPositionActionClaw(1),
                new SleepAction(.4),
                arm.moveToPositionActionArm(.5,.5),
                slides.moveSlidesToHeightAction(49, 1)
        );
        Action Pick2 = new SequentialAction(
                arm.moveToPositionActionArm(1,1),
                slides.moveSlidesToHeightAction(17, 1),//changing time here from .7 to .3 11/27/2024
                claw.setPositionActionClaw(1),
                new SleepAction(1),
                arm.moveToPositionActionArm(.5,.5),
                slides.moveSlidesToHeightAction(49, 1),
                new SleepAction(3),
                claw.setPositionActionClaw(0)
        );
        Action Pick3 = new SequentialAction(
                new SleepAction(.4),
                arm.moveToPositionActionArm(1,1),
                slides.moveSlidesToHeightAction(17, 1),
                claw.setPositionActionClaw(1),
                new SleepAction(.5),
                arm.moveToPositionActionArm(.5,.5),
                slides.moveSlidesToHeightAction(49, 1));
        Action MoveBackArm= new SequentialAction(
                claw.setPositionActionClaw(0),
                new SleepAction(.1),
                arm.moveToPositionActionArm(0,0));
        Action Block1Alt = new SequentialAction( new ParallelAction (
                arm.moveToPositionActionArm(.5,.5),
                slides.moveSlidesToHeightAction(27.5,1)),
                new SleepAction(.2),
                slides.moveSlidesToHeightAction(21.5,1),
                new SleepAction(.2),
                claw.setPositionActionClaw(0)
        );
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
        Action ClawOpen = claw.setPositionActionClaw(0);


        Pose2d initialPose = new Pose2d(8.5, 61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterDisp(0, Block1Alt)
                .strafeTo(new Vector2d(9, 35))// (37, 63) -> (6, 32)
                .waitSeconds(1)
                .strafeTo(new Vector2d(9, 39))// (30, 63) -> (6, 39)
                .afterDisp(0, Pick)
                .strafeTo(new Vector2d(50, 44))
                .waitSeconds(3.5) //Changed from 4.3 to 3.5 11/29/2024
                .afterTime(1,ClawOpen)
                .strafeToLinearHeading(new Vector2d(52, 56), 5*Math.PI/18, new TranslationalVelConstraint(40.0), new ProfileAccelConstraint(-10.0, 30))
                .waitSeconds(1)
                .afterDisp(5, Pick2)
                .strafeToLinearHeading(new Vector2d(58.5, 44),  Math.toRadians(268))
                .waitSeconds(5.3)
                .strafeToLinearHeading(new Vector2d(51.7, 56), 5*Math.PI/18)
                .waitSeconds(1)
                .afterDisp(0, Pick3)
                .strafeToLinearHeading(new Vector2d(58.2, 41.1),  Math.toRadians(308))
                .waitSeconds(5)
                .afterDisp(10,MoveBackArm)
                .strafeToLinearHeading(new Vector2d(51, 56), 5*Math.PI/18)
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(25,5),Math.PI);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        ClawClose,
                        new ParallelAction(
                                tab1.build()
                        )
                )
        );



    }
}