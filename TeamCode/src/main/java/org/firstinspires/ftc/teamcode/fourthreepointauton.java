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

@Autonomous(name = "RightSide2")
public class fourthreepointauton extends LinearOpMode {

    private Slides slides; // Declare slides
    private Arm arm;
    private Claw claw;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap);
        slides = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);
        Action Block = new SequentialAction(
                claw.setPositionActionClaw(1),
                slides.moveSlidesToHeightAction(27.5, 1),
                arm.moveToPositionActionArm(.5,.5),
                new SleepAction(.5),
                slides.moveSlidesToHeightAction(22, .75),
                new SleepAction(.5),
                claw.setPositionActionClaw(0),
                slides.moveSlidesToHeightAction(24, .75)
        );

        Action Pick = new SequentialAction(
                arm.moveToPositionActionArm(1,1),
                new SleepAction(2),
                slides.moveSlidesToHeightAction(17, .75),
                new SleepAction(1),
                claw.setPositionActionClaw(1),
                new SleepAction(1),
                arm.moveToPositionActionArm(.5,.5),
                slides.moveSlidesToHeightAction(49, 1),
                new SleepAction(.5),
                claw.setPositionActionClaw(0)
        );
        Action Pick2 = new SequentialAction(
                slides.moveSlidesToHeightAction(17, .75),
                arm.moveToPositionActionArm(1,1),
                new SleepAction(3),
                slides.moveSlidesToHeightAction(17, .75),
                claw.setPositionActionClaw(1),
                new SleepAction(1),
                arm.moveToPositionActionArm(.5,.5),
                slides.moveSlidesToHeightAction(49, 1),
                new SleepAction(3),
                claw.setPositionActionClaw(0),
                arm.moveToPositionActionArm(0,0)
        );
        Action Block1 = new SequentialAction(
                new ParallelAction (
                arm.moveToPositionActionArm(.5,.5),
                slides.moveSlidesToHeightAction(24,1)),
                new SleepAction(.5) ,
                claw.setPositionActionClaw(0)

        );
        Action Block1Alt = new SequentialAction( new ParallelAction (
                arm.moveToPositionActionArm(.5,.5),
                slides.moveSlidesToHeightAction(27.5,1)),
                new SleepAction(.5),
                slides.moveSlidesToHeightAction(22.5,1),
                new SleepAction(.2),
                claw.setPositionActionClaw(0));
        Action PickUp = new SequentialAction(
                new ParallelAction(
                        arm.moveToPositionActionArm(1,1),
                        new SleepAction(3),
                        slides.moveSlidesToHeightAction(17, .75)),
                        new SleepAction(.3),
                        claw.setPositionActionClaw(1)
                );
        Action PickUp3 = new SequentialAction(
                new ParallelAction(
                        arm.moveToPositionActionArm(1,1),
                        new SleepAction(1),
                        slides.moveSlidesToHeightAction(17, 1)),
                new SleepAction(.5),
                claw.setPositionActionClaw(1)
        );

        Action PickUp2 = new SequentialAction(
                new ParallelAction(
                        arm.moveToPositionActionArm(1,1),
                        slides.moveSlidesToHeightAction(20, 1)),
                new SleepAction(1),
                claw.setPositionActionClaw(1)
        );

        Action PickBlockSpec = new SequentialAction(
                new SleepAction(.5),
                arm.moveToPositionActionArm(1,1),

                slides.moveSlidesToHeightAction(17, 1),
                claw.setPositionActionClaw(1)
        );
        Action ArmTask3 = arm.moveToPositionActionArm(1,1);

        Action Everything = new SequentialAction(
                Block1Alt,
                new SleepAction(1),
                ArmTask3,
                new SleepAction(3),
                PickUp,
                new SleepAction(1),
                Block1Alt,
                new SleepAction(1),
                PickUp,
                new SleepAction(1),
                Block1Alt,
                new SleepAction(1),
                PickUp,
                new SleepAction(1),
                Block1Alt
        );
        Action slideTask = slides.moveSlidesToHeightAction(17, 1 );
        Action slideTask1 = slides.moveSlidesToHeightAction(22, .75 );
        Action slideTask2 = slides.moveSlidesToHeightAction(27.5, 1 );
        Action slideTask3 = slides.moveSlidesToHeightAction(45, 1 );
        Action slideTask4 = slides.moveSlidesToHeightAction(4000, 1 );
        Action ArmTask1 = arm.moveToPositionActionArm(0,0);
        Action ArmTask2 = arm.moveToPositionActionArm(.5,.5);
        Action ClawOpen = claw.setPositionActionClaw(0);
        Action ClawClose = claw.setPositionActionClaw(1);
        // myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9, 60, Math.toRadians(90)))
        // .strafeTo(new Vector2d(37, 63))
        // .strafeTo(new Vector2d(30, 63))
        // .strafeTo(new Vector2d(24, 30))
        // .turn((Math.toRadians(140)))
        // .strafeTo(new Vector2d(17.8, 14.5))
        // .turn((Math.toRadians(-135)))
        // .strafeTo(new Vector2d(12.5, 34))

        //Pose2d initialPose = new Pose2d(9, 61.5, Math.toRadians(270));
        Pose2d initialPose = new Pose2d(-15.15, 60, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize slides

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterDisp(0,Block1Alt)
                .strafeTo(new Vector2d(-9,35))
                .waitSeconds(1.3)
                .strafeTo(new Vector2d(-6,45))
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
                .afterDisp(15,PickBlockSpec)
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
                .strafeTo(new Vector2d(-35, 50));

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
        Actions.runBlocking(ClawClose);

        Actions.runBlocking(
                new SequentialAction(
                        // Parallel action: Execute the trajectory and slide/arm preparation
                        new ParallelAction(
                                tab1.build()



                                )
                        )
        );



    }
}
