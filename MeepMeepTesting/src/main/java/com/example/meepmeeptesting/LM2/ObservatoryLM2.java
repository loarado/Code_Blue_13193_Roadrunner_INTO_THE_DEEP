package com.example.meepmeeptesting.LM2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ObservatoryLM2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-24, 64.25, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(0,36,Math.toRadians(90)))
                        .splineToLinearHeading(new Pose2d(-33,48, Math.toRadians(270)),Math.toRadians(270))
                        .strafeTo(new Vector2d(-36,12))
                        .splineToLinearHeading(new Pose2d(-46,12,Math.toRadians(270)),Math.toRadians(90))
                        .strafeTo(new Vector2d(-46,53))
                        .strafeTo(new Vector2d(-46,18))
                        .splineToLinearHeading(new Pose2d(-55,12,Math.toRadians(270)),Math.toRadians(90))
                        .strafeTo(new Vector2d(-55,53))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}