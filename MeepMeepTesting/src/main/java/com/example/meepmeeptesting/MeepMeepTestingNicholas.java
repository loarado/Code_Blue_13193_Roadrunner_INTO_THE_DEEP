package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingNicholas {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(35, 60, Math.toRadians(-90)))
                        .strafeTo(new Vector2d(48,28 ))
                        .strafeTo(new Vector2d(48,38 ))
                        .splineTo(new Vector2d(55,55), Math.toRadians(45))
                        .strafeTo(new Vector2d(55,50))
                        .turn(Math.toRadians(45))
                        .splineTo(new Vector2d(58,28 ), Math.toRadians(-90))
                        .strafeTo(new Vector2d(58,38))
                        .splineTo(new Vector2d(55,55), Math.toRadians(45))
                        .strafeTo(new Vector2d(45,45))
                        .splineTo(new Vector2d(60,28), Math.toRadians(0))
                        .strafeTo(new Vector2d(50,30))
                        .splineTo(new Vector2d(55,55), Math.toRadians(45))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}