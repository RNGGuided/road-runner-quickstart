package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
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


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9, 61.5, Math.toRadians(270)))
                .strafeTo(new Vector2d(6, 35))// (37, 63) -> (6, 32)
                .waitSeconds(4)
                .strafeTo(new Vector2d(6, 39))// (30, 63) -> (6, 39)
                .strafeTo(new Vector2d(48, 40.5))// (24, 30) -> (39, 45)
                .waitSeconds(6)
                .turn(Math.toRadians(140))// Adjusted 140-degree turn
                .strafeTo(new Vector2d(48, 53))// (17.8, 14.5) -> (45.5, 51.2)
                .waitSeconds(3)
                .turn(Math.toRadians(-135))// Adjusted -135-degree turn
                .strafeTo(new Vector2d(58.5, 37)) // (12.5, 34) -> (34, 47.5)
                .waitSeconds(3)
                .turn(Math.toRadians(140))
                .strafeTo(new Vector2d(45.5,51.2))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}