package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 55, Math.toRadians(180), Math.toRadians(180), 15)
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


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8.5, 61.5, Math.toRadians(270)))
                        .strafeTo(new Vector2d(9, 35))// (37, 63) -> (6, 32)
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(9, 39))// (30, 63) -> (6, 39)
                        .strafeToLinearHeading(new Vector2d(50, 44),Math.PI/2)
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(52,56),5*Math.PI/18)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(58.3, 44),Math.PI/2)
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(52,56),5*Math.PI/18)
                        .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(58.7, 44),Math.toRadians(128))
                    .waitSeconds(2)
                    .strafeToLinearHeading(new Vector2d(52,56),5*Math.PI/18)
                        .waitSeconds(1)
                    .splineToLinearHeading(new Pose2d(30,0,Math.PI), Math.PI)
                .build());
                /*.strafeTo(new Vector2d(9, 35))// (37, 63) -> (6, 32)
                .waitSeconds(1)
                .strafeTo(new Vector2d(9, 39))// (30, 63) -> (6, 39)
                .strafeTo(new Vector2d(50, 44))
                .waitSeconds(3.5) //Changed from 4.3 to 3.5 11/29/2024
                .strafeToLinearHeading(new Vector2d(52, 56), 5*Math.PI/18, new TranslationalVelConstraint(40.0), new ProfileAccelConstraint(-10.0, 30))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(58.3, 44),  Math.toRadians(268))
                .waitSeconds(3.5)
                .strafeTo(new Vector2d(53, 44))
                .waitSeconds(1.4)
                .strafeToLinearHeading(new Vector2d(51.7, 56), 5*Math.PI/18)
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(58.7, 41.2),  Math.toRadians(308))
                .waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(51, 56), 5*Math.PI/18)
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(25,5),Math.PI)
                .build());*/

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}