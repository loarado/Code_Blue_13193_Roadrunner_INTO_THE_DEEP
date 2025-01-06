package com.example.meepmeeptesting.LM2;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BasketSamplesLM2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                        .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(36, 64.25, Math.toRadians(180)))
                        .setReversed(true)
                        .strafeTo(new Vector2d(36, 61))
                        .strafeToLinearHeading(new Vector2d(59, 52.5), Math.toRadians(258.5))
                        .waitSeconds(1)
                        .turn(Math.toRadians(11.5))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-11.5))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(40,12), Math.toRadians(0))
                        .strafeTo(new Vector2d(21,12))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}