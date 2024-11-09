package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

@Autonomous(name = "Drive Forward (48 inches) - Road Runner 1.0.x")
public class MainAutoMode extends LinearOpMode {

    private Slides slides; // Declare slides

    @Override
    public void runOpMode() throws InterruptedException {
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
        slides = new Slides(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(6, 32)) // (37, 63) -> (6, 32)
                .waitSeconds(2)
                .strafeTo(new Vector2d(6, 39))       // (30, 63) -> (6, 39)
                .strafeTo(new Vector2d(44, 40.5))    // (24, 30) -> (39, 45)
                .turn(Math.toRadians(140))          // Adjusted 140-degree turn
                .strafeTo(new Vector2d(48, 53))     // (17.8, 14.5) -> (45.5, 51.2)
                .turn(Math.toRadians(-135))         // Adjusted -135-degree turn
                .strafeTo(new Vector2d(58.5, 37));  // (12.5, 34) -> (34, 47.5)
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
                new SequentialAction(
                        new ParallelAction(
                                tab1.build()// Executes trajectory
                                // Moves slides in parallel
                        )
                )
        );
    }
}
