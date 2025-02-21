package com.example.meepmeeptesting.STATE;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Observatory4Spec_V5 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8, -64, Math.toRadians(275)))
                .setReversed(true)

                //place spec 1
              
                .splineToLinearHeading(new Pose2d(-3,-32.5, Math.toRadians(270)),Math.toRadians(90))
      
                .waitSeconds(0.7)
                .setReversed(false)

                //push samples
                .splineToLinearHeading(new Pose2d(26,-41, Math.toRadians(45)),Math.toRadians(270))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(30,-50), Math.toRadians(-45) )

                //move to sample 2
                .strafeToLinearHeading(new Vector2d(55,-12),Math.toRadians(-115))
                //push sample 2
                .strafeToLinearHeading(new Vector2d(55,-40),Math.toRadians(-90))


                //go to spec 2
                .splineToLinearHeading(new Pose2d(28,-59, Math.toRadians(270)),Math.toRadians(270))


                //pick spec 2
                .waitSeconds(.50)
                .strafeTo(new Vector2d(28, -63.5))
                .waitSeconds(0.7)

                //place spec 2
                //.strafeTo(new Vector2d(35, -60), )
                .setReversed(true)
                .strafeTo(new Vector2d(-1.5,-32) )
                .waitSeconds(0.7)
                .setReversed(false)

                //go to spec 3
                .splineToLinearHeading(new Pose2d(28.5,-45, Math.toRadians(270)),Math.toRadians(270))

                //pick spec 3

                .strafeTo(new Vector2d(28.5, -64))
                .waitSeconds(0.8)

                //place spec 3
                //.strafeTo(new Vector2d(35, -60), )
                .setReversed(true)
                .strafeTo(new Vector2d(0,-31))
                .waitSeconds(0.7)
                .setReversed(false)


                //go to spec 4
                .splineToLinearHeading(new Pose2d(28.5,-45, Math.toRadians(270)),Math.toRadians(270))

                //pick spec 4

                .strafeTo(new Vector2d(28.5, -64.5))

                .waitSeconds(0.8)

                //place spec 4
                //.strafeTo(new Vector2d(35, -60), )
                .setReversed(true)
                .strafeTo(new Vector2d(2,-30))
                .waitSeconds(0.7)
                .setReversed(false)

                /*
                //go to spec 5
                .afterTime(0.75,  new ParallelAction(specigrabber.SetPosition(var.speciArmGrab+grabChange), specigrabber.SpeciRotateGrab()))
                  .strafeTo(new Vector2d(33,-61.5), )

                //pick spec 5
                .waitSeconds(0.50)
                .strafeTo(new Vector2d(33, -65), )
                .afterTime(0.5, specigrabber.SpecigrabberClose())
                .afterTime(0.8, specigrabber.SetPosition(var.speciArmPrepScore))
                .afterTime(0.8, specigrabber.SpeciRotateScore())
                .waitSeconds(0.5)

                //place spec 5
                //.strafeTo(new Vector2d(35, -60), )
                .setReversed(true)
                .strafeTo(new Vector2d(-8,-32.5), )
                .afterTime(0.3,  specigrabber.SetPosition(var.speciArmScore))
                .afterTime(0.7, specigrabber.SpecigrabberOpen())
                .waitSeconds(0.7)
                .setReversed(false)
                */

                //park
                .splineToLinearHeading(new Pose2d(28,-53, Math.toRadians(325)),Math.toRadians(270))
                        .build());
        myBot.setDimensions(15, 16);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}