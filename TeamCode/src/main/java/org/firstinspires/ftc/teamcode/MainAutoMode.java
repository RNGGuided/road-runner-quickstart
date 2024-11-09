package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Slides;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Claw;

@Config
@Autonomous(name = "Main Auto Mode", group = "Autonomous")
public class MainAutoMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize subsystems
        Slides slides = new Slides(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        // Define actions
        Action slideTask = slides.moveSlidesToHeightAction(17, 1);
        Action slideTask1 = slides.moveSlidesToHeightAction(20, 1);
        Action slideTask2 = slides.moveSlidesToHeightAction(26.5, 1);
        Action slideTask3 = slides.moveSlidesToHeightAction(3000, 1);
        Action ArmTask2 = arm.moveToPositionActionArm(0.5, 0.5);
        Action ArmTask3 = arm.moveToPositionActionArm(1.0, 1.0);
        Action ClawOpen = claw.setPositionActionClaw(0);
        Action ClawClose = claw.setPositionActionClaw(1);

        // Initialize MecanumDrive
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(9, 61.5, Math.toRadians(270)));

        // Define trajectory
        Action trajectoryAction = drive.actionBuilder(new Pose2d(9, 61.5, Math.toRadians(270)))
                .afterDisp(10, slideTask2)
                .afterDisp(5, ArmTask2)
                .strafeTo(new Vector2d(6, 32))
                .afterDisp(0, slideTask1)
                .waitSeconds(1.5)
                .afterTime(2, ClawOpen)
                .waitSeconds(2.5)
                .strafeTo(new Vector2d(6, 39))
                .afterDisp(36, ArmTask3)
                .afterDisp(36, slideTask)
                .afterTime(3, ClawClose)
                .afterTime(4, ArmTask2)
                .waitSeconds(4.5)
                .strafeTo(new Vector2d(44, 40.5))
                .turn(Math.toRadians(140))
                .strafeTo(new Vector2d(48, 53))
                .turn(Math.toRadians(-135))
                .strafeTo(new Vector2d(58.5, 37))
                .build();

        // Pre-run telemetry
        telemetry.addLine("Robot Initialized. Waiting for Start.");
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Waiting for Start...");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        // Execute actions
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction, // Execute trajectory with integrated slide/arm/claw tasks
                        new ParallelAction( // Example of running actions in parallel
                                (packet) -> {
                                    telemetry.addLine("Parallel Action Running!");
                                    telemetry.update();
                                    return false;
                                },
                                ClawClose // Close the claw in parallel
                        ),
                        new Action() { // Custom action
                            @Override
                            public boolean run(@NonNull TelemetryPacket packet) {
                                telemetry.addLine("Custom Action Complete!");
                                telemetry.update();
                                return false; // Complete action immediately
                            }
                        }
                )
        );

        telemetry.addLine("Autonomous Complete.");
        telemetry.update();
    }
}

