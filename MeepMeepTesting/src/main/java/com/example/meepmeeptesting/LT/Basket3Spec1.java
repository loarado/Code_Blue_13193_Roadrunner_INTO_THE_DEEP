package com.example.meepmeeptesting.LT;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Basket3Spec1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-16, -64, Math.toRadians(275)))
                .setReversed(true)

                //place spec 1
              
                .splineToLinearHeading(new Pose2d(-14,-37.5, Math.toRadians(270)),Math.toRadians(90))
      
                .waitSeconds(0.7)
                .setReversed(false)

                .splineToLinearHeading(new Pose2d(-5,-32.5, Math.toRadians(270)),Math.toRadians(90))
                        .setReversed(false)
                .splineToLinearHeading(new Pose2d(-59, -52.5,Math.toRadians(90-14.5)), Math.toRadians(180))
                .turn(Math.toRadians(14.5))
                .turn(Math.toRadians(-14.5))
                .turn(Math.toRadians(14.5*2))
                .turn(Math.toRadians(-14.5*2))
                .splineToLinearHeading(new Pose2d(-21, -12,Math.toRadians(180)), Math.toRadians(0))
                        .build());
        myBot.setDimensions(15, 16);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}