package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.TESTING.Autonomous.SubsystemsPaths;

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
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Elbow;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Hand;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Lights;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Outtake;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Specigrabber;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.SubsystemsVariables;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Wrist;

@Disabled
@Autonomous(name = "Auto Observatory Test", group = "Autonomous")
public class BraulioAuto_Subsystems_Observatory extends LinearOpMode {
    public int distance = 0;

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(-24, 64.25, Math.toRadians(180));

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

        Outtake outtake = new Outtake(hardwareMap);

        Specigrabber specigrabber = new Specigrabber(hardwareMap);

        Actions.runBlocking(

                // MAKE SERVOS STAY IN PLACE AND STUFF

                new ParallelAction(
                        outtake.OuttakeIdle(),
                        specigrabber.SpecigrabberClose(),
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
                .splineTo(new Vector2d(54, 54), Math.toRadians(0));

        TrajectoryActionBuilder sample1 = drive.actionBuilder(beginPose)
                .setReversed(false)
                .strafeTo(new Vector2d(48, 48))
                .turnTo(Math.toRadians(-85));

        TrajectoryActionBuilder bucket2 = drive.actionBuilder(beginPose)
                .setReversed(true)
                .turnTo(Math.toRadians(45))
                .strafeTo(new Vector2d(54, 54));

        TrajectoryActionBuilder sample2 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(54, 48))
                .turnTo(Math.toRadians(-85))
                .strafeTo(new Vector2d(58.5, 48));

        TrajectoryActionBuilder bucket3 = drive.actionBuilder(beginPose)
                .turnTo(135)
                .strafeTo(new Vector2d(54, 54));

        TrajectoryActionBuilder park = drive.actionBuilder(beginPose)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(40,12), Math.toRadians(180))
                .lineToXLinearHeading(24, Math.toRadians(0));

        SequentialAction GrabSample = new SequentialAction(
                new ParallelAction(
                        hslide.HSlideToDist(340),
                        wrist.WristIntake(),
                        elbow.PrepElbowIntake(),
                        hand.HandIntake()
                ),
                new SleepAction(2),
                elbow.ElbowIntake(),
                new SleepAction(2),
                hand.HandStop(),
                new ParallelAction(
                        wrist.WristOuttake(),
                        elbow.ElbowOuttake(),
                        hslide.HSlideToTransfer()
                ),
                new SleepAction(2),
                hand.HandOuttake(),
                new SleepAction(2),
                hand.HandStop()
        );

        double sleepTime = 1.5;

        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    new SequentialAction(

                            new ParallelAction(
                                    bucket1.build(),
                                    vslides.VSlidesToDist(var.vSlideHighBasket,650)
                            ),
                            new SleepAction(sleepTime),
                            outtake.OuttakeOut(),
                            new SleepAction(sleepTime),
                            new ParallelAction(
                                    sample1.build(),
                                    vslides.VSlidesToDist(0, 650),
                                    outtake.OuttakeIdle()
                            ),
                            new SleepAction(sleepTime),

                            new SequentialAction(
                                    new ParallelAction(
                                            hslide.HSlideToDist(355),
                                            wrist.WristIntake(),
                                            elbow.PrepElbowIntake(),
                                            hand.HandIntake()
                                    ),
                                    new SleepAction(sleepTime),
                                    elbow.ElbowIntake(),
                                    new SleepAction(sleepTime),
                                    hand.HandStop(),
                                    new ParallelAction(
                                            wrist.WristOuttake(),
                                            elbow.ElbowOuttake(),
                                            hslide.HSlideToTransfer()
                                    ),
                                    new SleepAction(sleepTime),
                                    hand.HandOuttake(),
                                    new SleepAction(sleepTime),
                                    hand.HandStop()
                            ),

                            new ParallelAction(
                                    bucket2.build(),
                                    vslides.VSlidesToDist(var.vSlideHighBasket,650)
                            ),
                            new SleepAction(sleepTime),
                            outtake.OuttakeOut(),
                            new SleepAction(sleepTime),
                            new ParallelAction(
                                    sample2.build(),
                                    vslides.VSlidesToDist(0, 650),
                                    outtake.OuttakeIdle()
                            ),
                            new SleepAction(sleepTime),

                            new SequentialAction(
                                    new ParallelAction(
                                            hslide.HSlideToDist(355),
                                            wrist.WristIntake(),
                                            elbow.PrepElbowIntake(),
                                            hand.HandIntake()
                                    ),
                                    new SleepAction(sleepTime),
                                    elbow.ElbowIntake(),
                                    new SleepAction(sleepTime),
                                    hand.HandStop(),
                                    new ParallelAction(
                                            wrist.WristOuttake(),
                                            elbow.ElbowOuttake(),
                                            hslide.HSlideToTransfer()
                                    ),
                                    new SleepAction(sleepTime),
                                    hand.HandOuttake(),
                                    new SleepAction(sleepTime),
                                    hand.HandStop()
                            ),

                            new ParallelAction(
                                    bucket3.build(),
                                    vslides.VSlidesToDist(var.vSlideHighBasket,650)
                            ),
                            new SleepAction(sleepTime),
                            outtake.OuttakeOut(),
                            new SleepAction(sleepTime),
                            new ParallelAction(
                                    park.build(),
                                    vslides.VSlidesToDist(1000,650),
                                    outtake.OuttakeIdle()
                            )

                    )
            );
        }
    }
