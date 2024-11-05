package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingBasket {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(36, 64.25, Math.toRadians(180)))
                        .setReversed(true)
                        .strafeTo(new Vector2d(40, 60))
                        .splineTo(new Vector2d(54, 54), Math.toRadians(0))
                        .waitSeconds(1)
                        .setReversed(false)
                        .strafeTo(new Vector2d(48.5, 48))
                        .turn(Math.toRadians(45))
                        .waitSeconds(1)
                        .setReversed(true)
                        .splineTo(new Vector2d(54, 54), Math.toRadians(45))
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

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}