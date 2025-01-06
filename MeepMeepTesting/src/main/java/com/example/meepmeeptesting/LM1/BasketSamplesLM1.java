package com.example.meepmeeptesting.LM1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class BasketSamplesLM1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(36, 64.25, Math.toRadians(180)))
                        .setReversed(true)
                        .strafeTo(new Vector2d(40, 60))
                        .splineTo(new Vector2d(56, 56), Math.toRadians(0))
                        .waitSeconds(1)
                        .setReversed(false)
                        .strafeTo(new Vector2d(48, 48))
                        .turn(Math.toRadians(47))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-47))
                        .strafeTo(new Vector2d(56, 56))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(40,12), Math.toRadians(0))
                        .strafeTo(new Vector2d(21,12))
                        .build());
//test

        /*
        .waitSeconds(2)
                        .turn(Math.toRadians(-45))
                        .strafeTo(new Vector2d(54,54))
                        .waitSeconds(2)
                        .strafeTo(new Vector2d(54, 50))
                        .turn(Math.toRadians(45))
                        .strafeTo(new Vector2d(58, 40))
                        .strafeTo(new Vector2d(54,40))
                        .turn(Math.toRadians(-45))
                        .strafeTo(new Vector2d(54,54))
                        .waitSeconds(2)
                        .strafeTo(new Vector2d(54, 50))
                        .turn(Math.toRadians(135))
                        .strafeTo(new Vector2d(58,28))
                        .waitSeconds(2)
                        .strafeTo(new Vector2d(50,28))
                        .turn(Math.toRadians(225))
                        .strafeTo(new Vector2d(54,54))
                        .waitSeconds(2)
                        .splineTo(new Vector2d(40,12), Math.toRadians(180))
                        .strafeTo(new Vector2d(24,12))
                        .waitSeconds(2)
         */

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}