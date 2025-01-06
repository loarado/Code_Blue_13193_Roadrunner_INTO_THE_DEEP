package com.example.meepmeeptesting.LM2;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ObservatoryLM2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                        .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-24, 64.25, Math.toRadians(270)))
                        .splineToLinearHeading(new Pose2d(-33,48, Math.toRadians(270)),Math.toRadians(270))
                        .splineTo(new Vector2d(-36,12),Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-46,12,Math.toRadians(270)),Math.toRadians(90))
                        .strafeTo(new Vector2d(-46,53))
                        .strafeTo(new Vector2d(-46,18))
                        .splineToLinearHeading(new Pose2d(-55,12,Math.toRadians(270)),Math.toRadians(90))
                        .strafeTo(new Vector2d(-55,53))
                        .strafeTo(new Vector2d(-55,18))
                        .splineToLinearHeading(new Pose2d(-61,12,Math.toRadians(270)),Math.toRadians(90))
                        .strafeTo(new Vector2d(-61,53))
                        .strafeTo(new Vector2d(-61,40))
                        /*delay to allow human player to get samples out of observation zone before parking*/
                        .strafeTo(new Vector2d(-61,63))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}