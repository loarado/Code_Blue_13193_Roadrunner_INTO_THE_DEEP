package com.example.meepmeeptesting.LM2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BasketSamplesLM2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(36, 64.25, Math.toRadians(180)))
                        .setReversed(true)
                        .lineTo(new Vector2d(36, 61))
                        .lineToLinearHeading(new Pose2d(59, 53.5, Math.toRadians(247)))
                        .waitSeconds(1)
                        .turn(Math.toRadians(23))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-23))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(40,12, Math.toRadians(0)))
                        .strafeTo(new Vector2d(21,12))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}