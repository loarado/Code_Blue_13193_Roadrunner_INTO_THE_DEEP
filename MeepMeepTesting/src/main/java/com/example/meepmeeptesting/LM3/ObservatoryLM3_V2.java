package com.example.meepmeeptesting.LM3;


import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ObservatoryLM3_V2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(75, 75, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(24, -64, Math.toRadians(275)))
                .setReversed(true)

                //place spec 1
                .splineToLinearHeading(new Pose2d(5,-33, Math.toRadians(270)),Math.toRadians(90))
                .waitSeconds(0.8)
                .setReversed(false)

                //push samples
                .splineToLinearHeading(new Pose2d(26.5,-41, Math.toRadians(45)),Math.toRadians(270))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(29,-48), Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(31,-42, Math.toRadians(45)),Math.toRadians(235))
                .strafeTo(new Vector2d(37.5, -42))
                .strafeToLinearHeading(new Vector2d(40,-48),Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(40,-41, Math.toRadians(45)),Math.toRadians(235))
                .strafeTo(new Vector2d(46.25, -41))
                .strafeToLinearHeading(new Vector2d(44.5,-49), Math.toRadians(-45))

                //go to spec 2
                .splineToLinearHeading(new Pose2d(33,-61.5, Math.toRadians(270)),Math.toRadians(270))

                //pick spec 2
                .waitSeconds(.50)
                .strafeTo(new Vector2d(33, -64))
                .waitSeconds(1)

                //place spec 2
                //.strafeTo(new Vector2d(35, -60))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(1,-33),Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(false)
                //go to spec 3
                .splineToConstantHeading(new Vector2d(33,-61.5),Math.toRadians(270))

                //pick spec 3
                .waitSeconds(0.50)
                .strafeTo(new Vector2d(33, -64))
                .waitSeconds(1)

                //place spec 3
                //.strafeTo(new Vector2d(35, -60))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-2,-33),Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(false)
                //go to spec 4
                .splineToConstantHeading(new Vector2d(33,-61.5),Math.toRadians(270))

                //pick spec 4
                .waitSeconds(0.50)
                .strafeTo(new Vector2d(33, -64.3))
                .waitSeconds(1)

                //place spec 4
                //.strafeTo(new Vector2d(35, -60))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-5,-33),Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(false)
                //go to spec 5
                .splineToConstantHeading(new Vector2d(33,-61.5),Math.toRadians(270))

                //pick spec 5
                .waitSeconds(0.50)
                .strafeTo(new Vector2d(33, -64.3))
       
                .waitSeconds(1)

                //place spec 5
                //.strafeTo(new Vector2d(35, -60))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-8,-33),Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(false)
             


                //park
                
                .splineToLinearHeading(new Pose2d(25,-50, Math.toRadians(325)),Math.toRadians(270))
                        .build());
        myBot.setDimensions(15, 16);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}