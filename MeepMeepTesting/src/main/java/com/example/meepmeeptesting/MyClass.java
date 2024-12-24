package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        /*myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9, 61.5, Math.toRadians(270)))
                      .strafeTo(new Vector2d(6, 32))       // (37, 63) -> (6, 32)
            .strafeTo(new Vector2d(6, 39))       // (30, 63) -> (6, 39)
          .strafeTo(new Vector2d(48, 39))      // (24, 30) -> (39, 45)
        .turn(Math.toRadians(140))            // Adjusted 140-degree turn
        .strafeTo(new Vector2d(45.5, 51.2))  // (17.8, 14.5) -> (45.5, 51.2)
        .turn(Math.toRadians(-140))           // Adjusted -135-degree turn
        .strafeTo(new Vector2d(58.2, 39))// (12.5, 34) -> (34, 47.5)
                        .turn(Math.toRadians(140))
                        .strafeTo(new Vector2d(45.5,51.2))
                .build());*/


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-15.15, 60, Math.toRadians(270)))
                .strafeTo(new Vector2d(-6,35))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-6,45))
                .splineToLinearHeading(new Pose2d(-33,36,Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-38,12,Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-46,12,Math.toRadians(180)), Math.toRadians(180))
                .strafeTo(new Vector2d(-46, 12))
                .strafeTo(new Vector2d(-46, 60), new TranslationalVelConstraint(60.0), new ProfileAccelConstraint(-60, 60))
                .strafeTo(new Vector2d(-46, 12))
                .splineToLinearHeading(new Pose2d(-57,12,Math.toRadians(180)), Math.toRadians(180))
                .strafeTo(new Vector2d(-57, 60))
                .strafeTo(new Vector2d(-35,56))
                .strafeTo(new Vector2d(-36,61))
                .strafeToLinearHeading(new Vector2d(-36,58.5), Math.PI)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-8,35,3*Math.PI/2),3*Math.PI/2)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-36,58.5), Math.PI)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-8,50), 3*Math.PI/2)
                .strafeTo(new Vector2d(-8, 35))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-36,58.5), Math.PI)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-8,50), 3*Math.PI/2)
                .strafeTo(new Vector2d(-8, 35))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-36,58.5), Math.PI)
                .build());
                /*.afterDisp(0,Block)
                .strafeTo(new Vector2d(-6,35))
                .strafeTo(new Vector2d(-36, 35))
                .strafeTo(new Vector2d(-36, 12))
                .strafeTo(new Vector2d(-48, 12))
                .strafeTo(new Vector2d(-48, 60))
                .strafeTo(new Vector2d(-48, 12))
                .strafeTo(new Vector2d(-57, 12))
                .strafeTo(new Vector2d(-57, 60))
                .strafeTo(new Vector2d(-35, 50))
                .turn(Math.toRadians(180))
                .turn(Math.toRadians(-180))
                .strafeTo(new Vector2d(-8, 35))
                .strafeTo(new Vector2d(-35, 50))
                .turn(Math.toRadians(180))
                .turn(Math.toRadians(-180))
                .strafeTo(new Vector2d(-8, 35))
                .strafeTo(new Vector2d(-35, 50))
                .turn(Math.toRadians(180))
                .turn(Math.toRadians(-180))
                .strafeTo(new Vector2d(-8, 35))
                .strafeTo(new Vector2d(-35, 50))
                .build()); */

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}