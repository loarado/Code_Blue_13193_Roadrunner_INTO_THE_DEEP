package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.LEAGUE_MEET_1.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Elbow;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Hand;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Lights;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.OuttakeLM1;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Specigrabber;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Wrist;


@Disabled


@Autonomous(name = "LM1 - Basket 3 Samples not working :(", group = "Autonomous")
public class LM1_Basket_3_Samples_not_working extends LinearOpMode {
    public int distance = 0;

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(36, 64.25, Math.toRadians(180));

        // INSTANTIATE SUBSYSTEMS AND DT

        // I made all the subsystem classes check them out to understand how custom actions work!!!!

        SubsystemsVariables var = new SubsystemsVariables();

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        HorizontalSlides hslide = new HorizontalSlides(hardwareMap);

        VerticalSlides vslides = new VerticalSlides(hardwareMap);

        Elbow elbow = new Elbow(hardwareMap);

        Wrist wrist = new Wrist(hardwareMap);

        Hand hand = new Hand(hardwareMap);

        Lights lights = new Lights(hardwareMap);

        OuttakeLM1 outtakeLM1 = new OuttakeLM1(hardwareMap);

        Specigrabber specigrabber = new Specigrabber(hardwareMap);

        int hSlideGrabExtension = 375;

        int VSlideTempVelo = 1200;

        Actions.runBlocking(

                // MAKE SERVOS STAY IN PLACE AND STUFF

                new ParallelAction(
                        outtakeLM1.OuttakeIdle(),
                        specigrabber.SpecigrabberOpen(),
                        elbow.ElbowMiddle(),
                        wrist.WristMiddle(),
                        lights.LightsBlue(),
                        hand.HandStop(),
                        hslide.HSlideTo0()
                )
        );

        waitForStart();

        TrajectoryActionBuilder bucket1 = drive.actionBuilder(beginPose)
                .setReversed(true)
                .strafeTo(new Vector2d(40, 60))
                .splineTo(new Vector2d(56, 56), Math.toRadians(0));

        Pose2d bucket1end = new Pose2d(56, 56, Math.toRadians(225));

        TrajectoryActionBuilder sample1 = drive.actionBuilder(bucket1end)
                .setReversed(false)
                .strafeTo(new Vector2d(48, 48))
                .turn(Math.toRadians(47));

        Pose2d sample1end = new Pose2d(48, 48, Math.toRadians(272));

        TrajectoryActionBuilder bucket2 = drive.actionBuilder(sample1end)
                .turn(Math.toRadians(-47))
                .strafeTo(new Vector2d(56, 56));

        Pose2d bucket2end = new Pose2d(56, 56, Math.toRadians(225));


        TrajectoryActionBuilder sample2 = drive.actionBuilder(bucket2end)
                .strafeTo(new Vector2d(54, 48))
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(58.5, 48));

        Pose2d sample2end = new Pose2d(58.5, 48, Math.toRadians(270));

        TrajectoryActionBuilder bucket3 = drive.actionBuilder(sample2end)
                .turn(Math.toRadians(-45))
                .strafeTo(new Vector2d(56, 56));

        Pose2d bucket3end = new Pose2d(56, 56, Math.toRadians(225));

        TrajectoryActionBuilder park = drive.actionBuilder(bucket3end)
                .strafeToLinearHeading(new Vector2d(40,12), Math.toRadians(0))
                .strafeTo(new Vector2d(24,12));

        double sleepTime = 0.5;

        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    new SequentialAction(

                            new ParallelAction(
                                    bucket1.build(),
                                    vslides.VSlidesToDistPIDF(var.vSlideHighBasket)
                            ),
                            specigrabber.SpecigrabberClose(),
                            new SleepAction(2.5),
                            outtakeLM1.OuttakeOut(),
                            new SleepAction(1),
                            new ParallelAction(
                                    sample1.build(),
                                    vslides.VSlidesToDistPIDF(0),
                                    outtakeLM1.OuttakeIdle()
                            ),
                            new SleepAction(sleepTime),

                            new SequentialAction(
                                    new ParallelAction(
                                            hslide.HSlideToDist(hSlideGrabExtension),
                                            wrist.WristIntake(),
                                            elbow.PrepElbowIntake(),
                                            hand.HandIntake()
                                    ),
                                    new SleepAction(1),
                                    elbow.ElbowIntake(),
                                    new SleepAction(1),
                                    hand.HandStop(),
                                    new ParallelAction(
                                            wrist.WristTransfer(),
                                            elbow.ElbowTransfer(),
                                            hslide.HSlideToTransfer()
                                    ),
                                    new SleepAction(0.75),
                                    hand.HandOuttake(),
                                    new SleepAction(0.75),
                                    hand.HandStop()
                            ),

                            new ParallelAction(
                                    bucket2.build(),
                                    vslides.VSlidesToDistPIDF(var.vSlideHighBasket)
                            ),
                            new SleepAction(2.5),
                            outtakeLM1.OuttakeOut(),
                            new SleepAction(1),
                            new ParallelAction(
                                    sample2.build(),
                                    vslides.VSlidesToDistPIDF(0),
                                    outtakeLM1.OuttakeIdle()
                            ),
                            new SleepAction(sleepTime),

                            new SequentialAction(
                                    new ParallelAction(
                                            hslide.HSlideToDist(hSlideGrabExtension),
                                            wrist.WristIntake(),
                                            elbow.PrepElbowIntake(),
                                            hand.HandIntake()
                                    ),
                                    new SleepAction(1),
                                    elbow.ElbowIntake(),
                                    new SleepAction(1),
                                    hand.HandStop(),
                                    new ParallelAction(
                                            wrist.WristTransfer(),
                                            elbow.ElbowTransfer(),
                                            hslide.HSlideToTransfer()
                                    ),
                                    new SleepAction(0.75),
                                    hand.HandOuttake(),
                                    new SleepAction(0.75),
                                    hand.HandStop()
                            ),

                            new ParallelAction(
                                    bucket3.build(),
                                    vslides.VSlidesToDistPIDF(var.vSlideHighBasket)
                            ),
                            new SleepAction(2.5),
                            outtakeLM1.OuttakeOut(),
                            new SleepAction(sleepTime),
                            new ParallelAction(
                                    park.build(),
                                    vslides.VSlidesToDistPIDF(1030),
                                    outtakeLM1.OuttakeIdle()
                            )

                    )
            );
        }
    }
