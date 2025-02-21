package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.STATE.Autonomous;

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


@Autonomous(name = "| Observatory Spec V5 (Robot Push)", group = "Autonomous")
public class STATE_Observatory_Spec_V5 extends LinearOpMode {

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

        int grabChange = 10;

        VelConstraint tempVel = new TranslationalVelConstraint(38);
        AccelConstraint tempAccel = new ProfileAccelConstraint(-38, 38);

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


        TrajectoryActionBuilder pathActions = drive.actionBuilder(beginPose)

                //place spec 1
                .afterTime(0.5, new ParallelAction(
                        specigrabber.SetPosition(var.speciArmPrepScore),
                        specigrabber.SpeciRotateScore(),
                        vslides.VSlidesToDist(100)
                ))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-3,-32.5, Math.toRadians(270)),Math.toRadians(90), tempVel, tempAccel)
                .afterTime(0.15, specigrabber.SetPosition(var.speciArmScore))
                .afterTime(0.55, specigrabber.SpecigrabberOpen())
                .waitSeconds(0.55)
                .setReversed(false)


                //push samples
                .afterTime(1.3, new ParallelAction(specigrabber.SetPosition(var.speciArmGrab+grabChange), specigrabber.SpeciRotateGrab()))
                .afterTime(1, hslide.HSlideToMax())
                .afterTime(1.3, wrist.WristToDist(var.FrontIntakeWristPos))
                .afterTime(1.3, elbow.ElbowToDist(var.FrontIntakeElbowPos))
                .splineToLinearHeading(new Pose2d(26,-41, Math.toRadians(45)),Math.toRadians(270), tempVel, tempAccel)
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(30,-50), Math.toRadians(-45), tempVel, tempAccel)
                .waitSeconds(0.2)
                //push second sample with bot for more consistency
                .afterTime(0, hslide.HSlideTo0())
                .afterTime(0, wrist.WristMiddle())
                .afterTime(0, elbow.ElbowMiddle())
                //move to sample 2
                .strafeToLinearHeading(new Vector2d(55,-12),Math.toRadians(-115), tempVel, tempAccel)
                //push sample 2
                .strafeToLinearHeading(new Vector2d(55,-40),Math.toRadians(-90), tempVel, tempAccel)


                //go to spec 2
                .splineToLinearHeading(new Pose2d(28,-59, Math.toRadians(270)),Math.toRadians(270), tempVel, tempAccel)

                //pick spec 2
                .waitSeconds(.40)
                .strafeTo(new Vector2d(28, -63.5), tempVel, tempAccel)
                .afterTime(0.25, specigrabber.SpecigrabberClose())
                .afterTime(0.55, specigrabber.SetPosition(var.speciArmPrepScore))
                .afterTime(0.55, specigrabber.SpeciRotateScore())
                .waitSeconds(0.55)

                //place spec 2
                .setReversed(true)
                .strafeTo(new Vector2d(-1.5,-32), tempVel, tempAccel)
                .afterTime(0.15, specigrabber.SetPosition(var.speciArmScore))
                .afterTime(0.55, specigrabber.SpecigrabberOpen())
                .waitSeconds(0.55)
                .setReversed(false)

                //go to spec 3
                .afterTime(0,  specigrabber.SpeciRotateGrab())
                .afterTime(0.6,  new ParallelAction(specigrabber.SetPosition(var.speciArmGrab+grabChange), specigrabber.SpeciRotateGrab()))
                .splineToLinearHeading(new Pose2d(28.5,-59, Math.toRadians(270)),Math.toRadians(270), tempVel, tempAccel)

                //pick spec 3

                .waitSeconds(0.50)
                .strafeTo(new Vector2d(28.5, -64.25), tempVel, tempAccel)

                .afterTime(0.25, specigrabber.SpecigrabberClose())
                .afterTime(0.55, specigrabber.SetPosition(var.speciArmPrepScore))
                .afterTime(0.55, specigrabber.SpeciRotateScore())
                .waitSeconds(0.55)

                //place spec 3
                .setReversed(true)
                .strafeTo(new Vector2d(0,-31), tempVel, tempAccel)
                .afterTime(0.15, specigrabber.SetPosition(var.speciArmScore))
                .afterTime(0.55, specigrabber.SpecigrabberOpen())
                .waitSeconds(0.55)
                .setReversed(false)


                //go to spec 4
                .afterTime(0,  specigrabber.SpeciRotateGrab())
                .afterTime(0.6,  new ParallelAction(specigrabber.SetPosition(var.speciArmGrab+grabChange), specigrabber.SpeciRotateGrab()))
                .splineToLinearHeading(new Pose2d(28.5,-59, Math.toRadians(270)),Math.toRadians(270), tempVel, tempAccel)

                //pick spec 4

                .waitSeconds(0.50)
                .strafeTo(new Vector2d(28.5, -64.5), tempVel, tempAccel)

                .afterTime(0.25, specigrabber.SpecigrabberClose())
                .afterTime(0.55, specigrabber.SetPosition(var.speciArmPrepScore))
                .afterTime(0.55, specigrabber.SpeciRotateScore())
                .waitSeconds(0.55)

                //place spec 4
                .setReversed(true)
                .strafeTo(new Vector2d(2,-30), tempVel, tempAccel)
                .afterTime(0.15, specigrabber.SetPosition(var.speciArmScore))
                .afterTime(0.55, specigrabber.SpecigrabberOpen())
                .waitSeconds(0.55)
                .setReversed(false)

                //park
                .afterTime( 1.5,  new ParallelAction(specigrabber.SetPosition(var.speciArmGrab+grabChange), specigrabber.SpeciRotateGrab()))
                .afterTime(.1, hslide.HSlideToMax())
                .afterTime(.1, wrist.WristToDist(var.MiddleWristPos))
                .afterTime(.1, elbow.ElbowToDist(var.MiddleElbowPos))
                .splineToLinearHeading(new Pose2d(28,-53, Math.toRadians(325)),Math.toRadians(270), tempVel, tempAccel);


        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    new ParallelAction(
                            specigrabber.UpdatePID(),
                            pathActions.build()
                    )
            );
        }
    }
