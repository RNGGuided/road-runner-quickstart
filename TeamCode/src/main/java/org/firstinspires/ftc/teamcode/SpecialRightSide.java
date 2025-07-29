package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "SpecialSeperateRightSide2")
public class SpecialRightSide extends LinearOpMode {

    private Slides slides;
    private Arm arm;
    private Claw claw;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap);
        slides = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);

        // Define reusable actions
        Action Block1Alt = new SequentialAction(
                new ParallelAction(
                        arm.moveToPositionActionArm(0.5, 0.5),
                        slides.moveSlidesToHeightAction(27.5, 1)
                ),
                new SleepAction(0.5),
                slides.moveSlidesToHeightAction(22.5, 1),
                new SleepAction(0.2),
                claw.setPositionActionClaw(0)
        );

        Action PickUp = new SequentialAction(
                new ParallelAction(
                        arm.moveToPositionActionArm(1, 1),
                        slides.moveSlidesToHeightAction(17, 1)
                ),
                new SleepAction(0.3),
                claw.setPositionActionClaw(1)
        );

        Pose2d initialPose = new Pose2d(-15.15, 60, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // First trajectory: Initial positioning
        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .afterDisp(0, Block1Alt)
                .strafeTo(new Vector2d(-9, 35))
                .waitSeconds(1.3)
                .strafeTo(new Vector2d(-6, 45))
                .splineToLinearHeading(new Pose2d(-35, 36, Math.toRadians(180)), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-38, 12), Math.PI);

        // Second trajectory: Extended strafing
        TrajectoryActionBuilder traj2 = drive.actionBuilder(new Pose2d(-38, 12, Math.PI))
                .strafeTo(new Vector2d(-48, 12))
                .strafeTo(new Vector2d(-48, 60))
                .strafeTo(new Vector2d(-48, 12))
                .strafeTo(new Vector2d(-57, 12))
                .strafeTo(new Vector2d(-57, 55))
                .strafeTo(new Vector2d(-57, 50))
                .afterDisp(0, PickUp);

        // Third trajectory: Drop-off and return
        TrajectoryActionBuilder traj3 = drive.actionBuilder(new Pose2d(-57, 50, Math.PI))
                .strafeToLinearHeading(new Vector2d(-36, 50), Math.PI / 2)
                .waitSeconds(1.3)
                .strafeToLinearHeading(new Vector2d(-8, 50), 3 * Math.PI / 2)
                .afterDisp(2, Block1Alt)
                .strafeTo(new Vector2d(-8, 35))
                .waitSeconds(1)
                .afterDisp(0, PickUp);

        // Fourth trajectory: Repeat sequence
        TrajectoryActionBuilder traj4 = drive.actionBuilder(new Pose2d(-8, 35, Math.PI / 2))
                .strafeToLinearHeading(new Vector2d(-35, 50), Math.PI / 2)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-8, 50), 3 * Math.PI / 2)
                .afterDisp(0, Block1Alt)
                .strafeTo(new Vector2d(-8, 35))
                .waitSeconds(1)
                .afterDisp(0, PickUp)
                .strafeToLinearHeading(new Vector2d(-36.5, 50), Math.PI / 2)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-8, 50), 3 * Math.PI / 2)
                .afterDisp(0, Block1Alt)
                .strafeTo(new Vector2d(-8, 35))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-35, 50));

        waitForStart();

        // Close claw initially
        Actions.runBlocking(claw.setPositionActionClaw(1));

        // Execute trajectories sequentially
        Actions.runBlocking(new SequentialAction(
                traj1.build(),
                traj2.build(),
                traj3.build(),
                traj4.build()

        ));
    }
}
