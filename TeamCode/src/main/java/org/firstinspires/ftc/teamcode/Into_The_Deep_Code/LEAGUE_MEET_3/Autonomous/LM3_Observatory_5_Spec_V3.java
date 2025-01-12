package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.LEAGUE_MEET_3.Autonomous;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Elbow;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Lights;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Wrist;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM2_SUBSYSTEMS.OuttakeLM2;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM3_SUBSYSTEMS.HandLM3;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM3_SUBSYSTEMS.SpecigrabberLM3;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;


@Autonomous(name = "LM3 - Observatory 5 Spec V3", group = "Autonomous")
public class LM3_Observatory_5_Spec_V3 extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(8, -64, Math.toRadians(278));

        // INSTANTIATE SUBSYSTEMS AND DT

        // I made all the subsystem classes check them out to understand how custom actions work!!!!

        SubsystemsVariables var = new SubsystemsVariables();

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        HorizontalSlides hslide = new HorizontalSlides(hardwareMap);

        VerticalSlides vslides = new VerticalSlides(hardwareMap);

        Elbow elbow = new Elbow(hardwareMap);

        Wrist wrist = new Wrist(hardwareMap);

        HandLM3 handLM3 = new HandLM3(hardwareMap);

        Lights lights = new Lights(hardwareMap);

        OuttakeLM2 outtakeLM2 = new OuttakeLM2(hardwareMap);

        SpecigrabberLM3 specigrabber = new SpecigrabberLM3(hardwareMap);

        int grabChange = 25;

        VelConstraint tempVel = new TranslationalVelConstraint(55);
        AccelConstraint tempAccel = new ProfileAccelConstraint(-55, 55);

        Actions.runBlocking(

                // MAKE SERVOS STAY IN PLACE AND STUFF

                new ParallelAction(
                        outtakeLM2.OuttakeIdle(),
                        specigrabber.SpecigrabberClose(),
                        specigrabber.SpeciRotateScore(),
                        elbow.ElbowMiddle(),
                        wrist.WristMiddle(),
                        lights.LightsBlue(),
                        handLM3.HandStop(),
                        hslide.HSlideTo0()
                )
        );

        waitForStart();

/*
        TrajectoryActionBuilder path = drive.actionBuilder(beginPose)
               .setReversed(true)

                //place spec 1
                .splineToLinearHeading(new Pose2d(3,-33, Math.toRadians(270)),Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(false)

                //push samples
                .splineToLinearHeading(new Pose2d(24,-42, Math.toRadians(45)),Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(26.5,-48, Math.toRadians(-45)), Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(30,-42, Math.toRadians(45)),Math.toRadians(235))
                .strafeTo(new Vector2d(35, -42))
                .strafeToLinearHeading(new Vector2d(41,-48),Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(40,-42, Math.toRadians(45)),Math.toRadians(235))
                .strafeTo(new Vector2d(45.25, -42))
                .strafeToLinearHeading(new Vector2d(44,-49), Math.toRadians(-45))

                //go to spec 2
                .splineToLinearHeading(new Pose2d(35,-63, Math.toRadians(270)),Math.toRadians(270))

                //pick spec 2
                .waitSeconds(.75)
                .strafeTo(new Vector2d(35, -65.75))
                .waitSeconds(1)

                //place spec 2
                .strafeTo(new Vector2d(35, -60))
                .splineToLinearHeading(new Pose2d(1,-33, Math.toRadians(270)),Math.toRadians(90))
                .waitSeconds(1)
                //go to spec 3
                .splineToLinearHeading(new Pose2d(35,-63, Math.toRadians(270)),Math.toRadians(270))
                //pick spec 3
                .waitSeconds(0.75)
                .strafeTo(new Vector2d(35, -65.75))
                .waitSeconds(1)
                //place spec 3
                .strafeTo(new Vector2d(35, -60))
                .splineToLinearHeading(new Pose2d(-2,-33, Math.toRadians(270)),Math.toRadians(90))
                .waitSeconds(1)
                //go to spec 4
                .splineToLinearHeading(new Pose2d(35,-63, Math.toRadians(270)),Math.toRadians(270))
                //pick spec 4
                .waitSeconds(0.75)
                .strafeTo(new Vector2d(35, -66))
                .waitSeconds(1)
                //place spec 4
                .strafeTo(new Vector2d(35, -60))
                .splineToLinearHeading(new Pose2d(-5,-33, Math.toRadians(270)),Math.toRadians(90))
                .waitSeconds(1)
                //go to spec 5
                .splineToLinearHeading(new Pose2d(35,-61, Math.toRadians(270)),Math.toRadians(270))
                //pick spec 5
                .waitSeconds(0.75)
                .strafeTo(new Vector2d(35, -66))
                .waitSeconds(1)
                //place spec 5
                .strafeTo(new Vector2d(35, -60))
                .splineToLinearHeading(new Pose2d(-8,-33, Math.toRadians(270)),Math.toRadians(90))
                .waitSeconds(1)
                //park
                .splineToLinearHeading(new Pose2d(25,-50, Math.toRadians(335)),Math.toRadians(270))
*/



        TrajectoryActionBuilder pathActions = drive.actionBuilder(beginPose)
                .setReversed(true)

                //place spec 1
                .afterTime(0.5, new ParallelAction(
                        specigrabber.SetPosition(var.speciArmPrepScore),
                        specigrabber.SpeciRotateScore()
                ))
                .splineToLinearHeading(new Pose2d(3,-32.5, Math.toRadians(270)),Math.toRadians(90), tempVel, tempAccel)
                .afterTime(0.2, specigrabber.SetPosition(var.speciArmScore))
                .afterTime(0.7, specigrabber.SpecigrabberOpen())
                .waitSeconds(0.7)
                .setReversed(false)

                //push samples
                .afterTime(0.5, new ParallelAction(specigrabber.SetPosition(var.speciArmGrab+grabChange), specigrabber.SpeciRotateGrab()))

                .afterTime(1, hslide.HSlideToMax())
                .afterTime(1.3, wrist.WristToDist(var.FrontIntakeWristPos))
                .afterTime(1.3, elbow.ElbowToDist(var.FrontIntakeElbowPos))
                .splineToLinearHeading(new Pose2d(26,-41, Math.toRadians(45)),Math.toRadians(270))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(30,-50), Math.toRadians(-45), tempVel, tempAccel)
                .splineToLinearHeading(new Pose2d(31,-42, Math.toRadians(45)),Math.toRadians(235), tempVel, tempAccel)
                .strafeTo(new Vector2d(38.5, -42), tempVel, tempAccel)
                .strafeToLinearHeading(new Vector2d(42,-50),Math.toRadians(-45), tempVel, tempAccel)
                .splineToLinearHeading(new Pose2d(40,-41, Math.toRadians(45)),Math.toRadians(235), tempVel, tempAccel)
                .strafeTo(new Vector2d(46, -41), tempVel, tempAccel)
                .strafeToLinearHeading(new Vector2d(46,-50), Math.toRadians(-45), tempVel,tempAccel)

                //go to spec 2
                .afterTime(0, hslide.HSlideTo0())
                .afterTime(0, wrist.WristMiddle())
                .afterTime(0, elbow.ElbowMiddle())
                .splineToLinearHeading(new Pose2d(33,-59, Math.toRadians(270)),Math.toRadians(270), tempVel, tempAccel)

                //pick spec 2
                .waitSeconds(.50)
                .strafeTo(new Vector2d(33, -62.5), tempVel, tempAccel)
                .afterTime(0.4, specigrabber.SpecigrabberClose())
                .afterTime(0.7, specigrabber.SetPosition(var.speciArmPrepScore))
                .afterTime(0.7, specigrabber.SpeciRotateScore())
                .waitSeconds(0.7)

                //place spec 2
                //.strafeTo(new Vector2d(35, -60), tempVel, tempAccel)
                .setReversed(true)
                .strafeTo(new Vector2d(0.5,-32.5), tempVel, tempAccel)
                .afterTime(0.3, specigrabber.SetPosition(var.speciArmScore))
                .afterTime(0.7, specigrabber.SpecigrabberOpen())
                .waitSeconds(0.7)
                .setReversed(false)

                //go to spec 3
                .afterTime(0.75,  new ParallelAction(specigrabber.SetPosition(var.speciArmGrab+grabChange), specigrabber.SpeciRotateGrab()))
                .splineToLinearHeading(new Pose2d(33,-60, Math.toRadians(270)),Math.toRadians(270), tempVel, tempAccel)

                //pick spec 3
                .waitSeconds(0.50)
                .strafeTo(new Vector2d(33, -63.5), tempVel, tempAccel)
                .afterTime(0.5, specigrabber.SpecigrabberClose())
                .afterTime(0.8, specigrabber.SetPosition(var.speciArmPrepScore))
                .afterTime(0.8, specigrabber.SpeciRotateScore())
                .waitSeconds(0.8)

                //place spec 3
                //.strafeTo(new Vector2d(35, -60), tempVel, tempAccel)
                .setReversed(true)
                .strafeTo(new Vector2d(-2,-32), tempVel, tempAccel)
                .afterTime(0.3, specigrabber.SetPosition(var.speciArmScore))
                .afterTime(0.7, specigrabber.SpecigrabberOpen())
                .waitSeconds(0.7)
                .setReversed(false)

                //go to spec 4
                .afterTime(0.75,  new ParallelAction(specigrabber.SetPosition(var.speciArmGrab+grabChange), specigrabber.SpeciRotateGrab()))
                .splineToLinearHeading(new Pose2d(33,-60, Math.toRadians(270)),Math.toRadians(270), tempVel, tempAccel)

                //pick spec 4
                .waitSeconds(0.50)
                .strafeTo(new Vector2d(33, -63.5), tempVel, tempAccel)
                .afterTime(0.5, specigrabber.SpecigrabberClose())
                .afterTime(0.8, specigrabber.SetPosition(var.speciArmPrepScore))
                .afterTime(0.8, specigrabber.SpeciRotateScore())
                .waitSeconds(0.8)

                //place spec 4
                //.strafeTo(new Vector2d(35, -60), tempVel, tempAccel)
                .setReversed(true)
                .strafeTo(new Vector2d(-5,-32), tempVel, tempAccel)
                .afterTime(0.3, specigrabber.SetPosition(var.speciArmScore))
                .afterTime(0.7, specigrabber.SpecigrabberOpen())
                .waitSeconds(0.7)
                .setReversed(false)

                /*
                //go to spec 5
                .afterTime(0.75,  new ParallelAction(specigrabber.SetPosition(var.speciArmGrab+grabChange), specigrabber.SpeciRotateGrab()))
                  .strafeTo(new Vector2d(33,-61.5), tempVel, tempAccel)

                //pick spec 5
                .waitSeconds(0.50)
                .strafeTo(new Vector2d(33, -65), tempVel, tempAccel)
                .afterTime(0.5, specigrabber.SpecigrabberClose())
                .afterTime(0.8, specigrabber.SetPosition(var.speciArmPrepScore))
                .afterTime(0.8, specigrabber.SpeciRotateScore())
                .waitSeconds(0.5)

                //place spec 5
                //.strafeTo(new Vector2d(35, -60), tempVel, tempAccel)
                .setReversed(true)
                .strafeTo(new Vector2d(-8,-32.5), tempVel, tempAccel)
                .afterTime(0.3,  specigrabber.SetPosition(var.speciArmScore))
                .afterTime(0.7, specigrabber.SpecigrabberOpen())
                .waitSeconds(0.7)
                .setReversed(false)
                */

                //park
                .afterTime(0.75,  new ParallelAction(specigrabber.SetPosition(var.speciArmGrab+grabChange), specigrabber.SpeciRotateGrab()))
                .afterTime(1, hslide.HSlideToMax())
                .afterTime(1, wrist.WristToDist(var.FrontIntakeWristPos))
                .afterTime(1, elbow.ElbowToDist(var.FrontIntakeElbowPos))
                .splineToLinearHeading(new Pose2d(25,-50, Math.toRadians(325)),Math.toRadians(270), tempVel, tempAccel);


        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    new ParallelAction(
                            specigrabber.UpdatePID(),
                            pathActions.build()
                    )
            );
        }
    }
