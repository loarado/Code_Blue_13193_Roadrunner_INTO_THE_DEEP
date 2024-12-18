package com.example.meepmeeptesting.LM3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.ui.TrajectoryProgressSliderMaster;

public class ObservatoryLM3 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(24, -64.25, Math.toRadians(270)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(6,-31, Math.toRadians(270)),Math.toRadians(90))
                        .setReversed(false)
                        //push samples
                        .splineToLinearHeading(new Pose2d(24,-40, Math.toRadians(45)),Math.toRadians(270))
                        .lineToSplineHeading(new Pose2d(26,-43, Math.toRadians(-30)))
                        .splineToLinearHeading(new Pose2d(33,-40, Math.toRadians(45)),Math.toRadians(235))
                        .lineToSplineHeading(new Pose2d(35,-43, Math.toRadians(-30)))
                        .splineToLinearHeading(new Pose2d(42,-40, Math.toRadians(45)),Math.toRadians(235))
                        .lineToSplineHeading(new Pose2d(44,-43, Math.toRadians(-30)))

                        .splineToLinearHeading(new Pose2d(36,-61, Math.toRadians(90)),Math.toRadians(235))
                        .strafeTo(new Vector2d(36, -64.25))
                        .splineToLinearHeading(new Pose2d(4,-31, Math.toRadians(270)),Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(36,-61, Math.toRadians(90)),Math.toRadians(235))
                        .strafeTo(new Vector2d(36, -64.25))
                        .splineToLinearHeading(new Pose2d(2,-31, Math.toRadians(270)),Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(36,-61, Math.toRadians(90)),Math.toRadians(235))
                        .strafeTo(new Vector2d(36, -64.25))
                        .splineToLinearHeading(new Pose2d(0,-31, Math.toRadians(270)),Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(36,-61, Math.toRadians(90)),Math.toRadians(235))
                        .strafeTo(new Vector2d(36, -64.25))
                        .splineToLinearHeading(new Pose2d(-2,-31, Math.toRadians(270)),Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(36,-61, Math.toRadians(90)),Math.toRadians(235))
                        .build());
        myBot.setDimensions(15, 16);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}