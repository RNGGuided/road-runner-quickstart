package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(63, -14, 270))
                        .strafeToLinearHeading(new Vector2d(58, -14), Math.toRadians(325))
                        .waitSeconds(4)
                        .splineToLinearHeading(new Pose2d(60, -62, Math.toRadians(270)),Math.toRadians(90   ))
                        .strafeToLinearHeading(new Vector2d(59, -56), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(64, -62), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(59, -56), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(58, -14), Math.toRadians(248.5))
                        .waitSeconds(6)
        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}