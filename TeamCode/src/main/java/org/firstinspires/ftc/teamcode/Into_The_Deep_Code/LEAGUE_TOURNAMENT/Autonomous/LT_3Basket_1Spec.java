package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.LEAGUE_TOURNAMENT.Autonomous;

import static com.google.gson.internal.JsonReaderInternalAccess.INSTANCE;

import static org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive.pose;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Time;
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
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LT_SUBSYSTEMS.DistLT;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;

import kotlin.jvm.internal.Lambda;




@Autonomous(name = "LT - 3 Basket 1 Spec", group = "Autonomous")
public class LT_3Basket_1Spec extends LinearOpMode {

    @Override
    public void runOpMode() {




        Pose2d beginPose = new Pose2d(-16, -64, Math.toRadians(278));

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

        DistLT distance = new DistLT(hardwareMap);

        int grabChange = 10;
        int hSlideGrabExtension = var.hSlideRuleMax;
        int sleepTime = 1;

        VelConstraint tempVel = new TranslationalVelConstraint(42);
        AccelConstraint tempAccel = new ProfileAccelConstraint(-50, 90);

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


        TrajectoryActionBuilder initialDrive = drive.actionBuilder(beginPose)
                .setReversed(true)
                .afterTime(0.5, new ParallelAction(
                        specigrabber.SetPosition(var.speciArmPrepScore),
                        specigrabber.SpeciRotateScore(),
                        vslides.VSlidesToDist(100)
                ))
                .splineToLinearHeading(new Pose2d(-14,-37.5, Math.toRadians(270)),Math.toRadians(90));


        TrajectoryActionBuilder restDrive = drive.actionBuilder(new Pose2d(-14, 37.5, Math.toRadians(270)))


                .splineToLinearHeading(new Pose2d(-3,-32.5, Math.toRadians(270)),Math.toRadians(90), tempVel, tempAccel)
                .afterTime(0.2, specigrabber.SetPosition(var.speciArmScore))
                .afterTime(0.7, specigrabber.SpecigrabberOpen())
                .waitSeconds(1)
                .setReversed(false)

                .afterTime(1.3, new ParallelAction(specigrabber.SetPosition(var.speciArmGrab+grabChange), specigrabber.SpeciRotateGrab()))
                .setReversed(false)
                .afterTime(1.3, vslides.SetPosition(0))
                //prepare to intake sample ONE
                .afterTime(1, hslide.HSlideToDist(var.hSlideRuleMax))
                .afterTime(1, wrist.WristIntake())
                .afterTime(2, elbow.PrepElbowIntake())
                .afterTime(2, handLM3.HandIntake())
                .splineToLinearHeading(new Pose2d(-59, -52.5,Math.toRadians(90-14.5)), Math.toRadians(180))
                .waitSeconds(sleepTime)

                //score sample number ONE

                //grab sample TWO and bring back the outtake
                .afterTime(0, elbow.ElbowIntake())
                .afterTime(0, outtakeLM2.OuttakeIdle())
                .waitSeconds(sleepTime)

                //lower slides back to successfully transfer the next sample
                .afterTime(0,vslides.SetPosition(0))
                .waitSeconds(sleepTime)

                //prepare to transfer sample TWO to the outtake
                .afterTime(0, handLM3.HandStop())
                .afterTime(0, wrist.WristTransfer())
                .afterTime(0, elbow.ElbowTransfer())
                .afterTime(0, hslide.HSlideToTransfer())
                .waitSeconds(0.75)

                //transfer sample TWO

                //this is only to be safe, if the wrist and elbow can
                //be in transfer position while the slides come down
                //just do that instead
                .afterTime(0.5, handLM3.HandOuttake())
                .waitSeconds(sleepTime)

                //prepare to intake sample THREE while scoring sample TWO
                .afterTime(0, handLM3.HandStop())
                .afterTime(0, hslide.HSlideToDist(hSlideGrabExtension-45))
                .afterTime(0, wrist.WristIntake())
                .afterTime(0, elbow.ElbowMiddle())
                .afterTime(0, vslides.SetPosition(2377))
                .waitSeconds(1.25)

                //score sample TWO
                .afterTime(0, outtakeLM2.OuttakeOut())
                .waitSeconds(sleepTime)

                //reset outtake after scoring sample 2
                .afterTime(0, outtakeLM2.OuttakeIdle())

                //move to next scoring position
                .turn(Math.toRadians(14.5))
                .waitSeconds(0.75)

                //lower slides to transfer sample 3
                .afterTime(0, vslides.SetPosition(0))
                .waitSeconds(0.5)

                //prepare to grab sample 3
                .afterTime(0, elbow.PrepElbowIntake())
                .afterTime(0, handLM3.HandIntake())
                .waitSeconds(sleepTime)

                //grab sample 3
                .afterTime(0, elbow.ElbowIntake())
                .waitSeconds(sleepTime)

                //prepare for transfer
                .afterTime(0, handLM3.HandStop())
                .afterTime(0, wrist.WristTransfer())
                .afterTime(0, elbow.ElbowTransfer())
                .afterTime(0, hslide.HSlideToTransfer())
                .waitSeconds(0.75)

                //transfer sample 3
                .afterTime(0.5, handLM3.HandOuttake())
                .waitSeconds(1)



                //prepare to intake sample FOUR while scoring sample THREE
                .afterTime(0, handLM3.HandStop())
                .afterTime(0, wrist.WristIntake())
                .afterTime(0, elbow.ElbowMiddle())
                .afterTime(0, hslide.HSlideToDist(hSlideGrabExtension-5))
                .afterTime(0, vslides.SetPosition(2377))
                .turn(Math.toRadians(-14.5))
                .waitSeconds(1)

                //score sample 3
                .afterTime(0, outtakeLM2.OuttakeOut())
                .waitSeconds(sleepTime)

                //reset outtake after scoring sample 3
                .afterTime(0, outtakeLM2.OuttakeIdle())

                //move to next scoring position
                .turn(Math.toRadians(40))
                .waitSeconds(0.75)

                //lower slides to transfer sample 4
                .afterTime(0, vslides.SetPosition(0))
                .waitSeconds(0.5)

                //prepare to grab sample 4
                .afterTime(0, elbow.PrepElbowIntake())
                .afterTime(0, handLM3.HandIntake())
                .waitSeconds(sleepTime)

                //grab sample 4
                .afterTime(0, elbow.ElbowIntake())
                .waitSeconds(sleepTime)

                //prepare for transfer
                .afterTime(0, handLM3.HandStop())
                .afterTime(0, wrist.WristTransfer())
                .afterTime(0, elbow.ElbowTransfer())
                .afterTime(0, hslide.HSlideToTransfer())
                .waitSeconds(0.75)

                //transfer sample 4
                .afterTime(0.5, handLM3.HandOuttake())
                .waitSeconds(1)



                //prepare to score sample 4
                .afterTime(0, handLM3.HandStop())
                .afterTime(0, wrist.WristIntake())
                .afterTime(0, elbow.ElbowMiddle())
                .afterTime(0, vslides.SetPosition(2377))
                .turn(Math.toRadians(-40))
                .waitSeconds(1)

                //score sample 4
                .afterTime(0, outtakeLM2.OuttakeOut())
                .waitSeconds(sleepTime)

                //reset outtake and lower slides
                .afterTime(0, outtakeLM2.OuttakeIdle())
                .waitSeconds(sleepTime)
                .afterTime(0, vslides.SetPosition(735))
                //park in the designated area
                .strafeToLinearHeading(new Vector2d(40,12), Math.toRadians(0))
                .strafeTo(new Vector2d(21,12));




        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    new ParallelAction(
                            specigrabber.UpdatePID(),
                            vslides.UpdatePID(),
                            new SequentialAction(
                             initialDrive.build(),
                                    distance.DistGreater(),
                                    restDrive.build()
                            )
                    )
            );
        }
    }


