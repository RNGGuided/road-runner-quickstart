package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

@Autonomous(name = "Drive Forward (48 inches) - Road Runner 1.0.x")
public class MainAutoMode extends LinearOpMode {

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
                slides.moveSlidesToHeightAction(27.5, 1 ),
                new SleepAction(1),
                slides.moveSlidesToHeightAction(22, .75 ),
                new SleepAction(.5),
                claw.setPositionActionClaw(0)
        );
        Action slideTask = slides.moveSlidesToHeightAction(17, 1 );
        Action slideTask1 = slides.moveSlidesToHeightAction(22, .75 );
        Action slideTask2 = slides.moveSlidesToHeightAction(27.5, 1 );
        Action slideTask3 = slides.moveSlidesToHeightAction(45, 1 );
        Action slideTask4 = slides.moveSlidesToHeightAction(4000, 1 );
        Action ArmTask1 = arm.moveToPositionActionArm(0,0);
        Action ArmTask2 = arm.moveToPositionActionArm(.5,.5);
        Action ArmTask3 = arm.moveToPositionActionArm(1,1);
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

        Pose2d initialPose = new Pose2d(9, 61.5, Math.toRadians(270));
        // Pose2d initialPose = new Pose2d(-12, 60, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize slides

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterDisp(10, ArmTask2)
                .strafeTo(new Vector2d(6, 33))// (37, 63) -> (6, 32)
                .waitSeconds(4)
                .strafeTo(new Vector2d(6, 39))// (30, 63) -> (6, 39)
                .afterDisp(36, slideTask)
                .afterTime(2, ClawClose)
                .strafeTo(new Vector2d(44, 40.5))// (24, 30) -> (39, 45)
                .turn(Math.toRadians(140))// Adjusted 140-degree turn
                .afterTime(2, slideTask3)
                .afterTime(2,ClawOpen)
                .waitSeconds(2)
                .strafeTo(new Vector2d(48, 53))// (17.8, 14.5) -> (45.5, 51.2)
                .waitSeconds(4)
                .afterTime(3, ClawOpen)
                .turn(Math.toRadians(-135))         // Adjusted -135-degree turn
                .strafeTo(new Vector2d(58.5, 37)) // (12.5, 34) -> (34, 47.5)
                .turn(Math.toRadians(140))
                .strafeTo(new Vector2d(45.5,51.2));
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
                                tab1.build(),
                                Block// Execute the trajectory
                                new SequentialAction(
                                        slideTask2,            // Move slides to 26.5 inches
                                        slideTask1,            // Move slides to 20 inches
                                        new SleepAction(1),    // Wait for 1 second
                                        ClawOpen,              // Open the claw
                                        new SleepAction(2),    // Wait for 2 seconds
                                        ArmTask3,
                                        new SleepAction(1),// Move arm to position 1// Wait for 1 second after ArmTask3
                                        ArmTask2,
                                        new SleepAction(1) // Move arm back to position 0.5

                                )
                        )
                )
        );



    }
}
