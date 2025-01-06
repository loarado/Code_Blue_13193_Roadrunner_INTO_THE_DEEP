package com.example.meepmeeptesting.LM3;


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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(24, -64.25, Math.toRadians(270)))
                .setReversed(true)

                //place spec 1
                .splineToLinearHeading(new Pose2d(3,-33, Math.toRadians(270)),Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(false)

                //push samples
                .splineToLinearHeading(new Pose2d(24,-42, Math.toRadians(45)),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(26.5,-48), Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(30,-42, Math.toRadians(45)),Math.toRadians(235))
                .strafeTo(new Vector2d(37.5, -42))
                .strafeToLinearHeading(new Vector2d(39,-48),Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(40,-42, Math.toRadians(45)),Math.toRadians(235))
                .strafeTo(new Vector2d(45.25, -42))
                .strafeToLinearHeading(new Vector2d(44,-49), Math.toRadians(-45))

                //go to spec 2
                .splineToLinearHeading(new Pose2d(37.5,-63, Math.toRadians(90)),Math.toRadians(235))

                //pick spec 2
                .waitSeconds(.75)
                .strafeTo(new Vector2d(37.5, -65.75))
                .waitSeconds(1)

                //place spec 2
                .strafeTo(new Vector2d(37.5, -60))
                .splineToLinearHeading(new Pose2d(1,-33, Math.toRadians(270)),Math.toRadians(90))
                .waitSeconds(1)
                //go to spec 3
                .splineToLinearHeading(new Pose2d(37.5,-63, Math.toRadians(90)),Math.toRadians(235))
                //pick spec 3
                .waitSeconds(0.75)
                .strafeTo(new Vector2d(37.5, -65.75))
                .waitSeconds(1)
                //place spec 3
                .strafeTo(new Vector2d(37.5, -60))
                .splineToLinearHeading(new Pose2d(-2,-33, Math.toRadians(270)),Math.toRadians(90))
                .waitSeconds(1)
                //go to spec 4
                .splineToLinearHeading(new Pose2d(37.5,-63, Math.toRadians(90)),Math.toRadians(235))
                //pick spec 4
                .waitSeconds(0.75)
                .strafeTo(new Vector2d(37.5, -66))
                .waitSeconds(1)
                //place spec 4
                .strafeTo(new Vector2d(37.5, -60))
                .splineToLinearHeading(new Pose2d(-5,-33, Math.toRadians(270)),Math.toRadians(90))
                .waitSeconds(1)
                //go to spec 5
                .splineToLinearHeading(new Pose2d(37.5,-61, Math.toRadians(90)),Math.toRadians(235))
                //pick spec 5
                .waitSeconds(0.75)
                .strafeTo(new Vector2d(37.5, -66))
                .waitSeconds(1)
                //place spec 5
                .strafeTo(new Vector2d(37.5, -60))
                .splineToLinearHeading(new Pose2d(-8,-33, Math.toRadians(270)),Math.toRadians(90))
                .waitSeconds(1)
                //park
                .splineToLinearHeading(new Pose2d(37.5,-62, Math.toRadians(90)),Math.toRadians(235))
                        .build());
        myBot.setDimensions(15, 16);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}