package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.LEAGUE_MEET_1.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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



@Autonomous(name = "Basket 3 Samples not working :(", group = "Autonomous")
public class Basket_3_Samples_not_working extends LinearOpMode {
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

        Outtake outtake = new Outtake(hardwareMap);

        Specigrabber specigrabber = new Specigrabber(hardwareMap);

        int hSlideGrabExtension = 370;

        int VSlideTempVelo = 950;

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

        double sleepTime = 1;

        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    new SequentialAction(

                            new ParallelAction(
                                    bucket1.build(),
                                    vslides.VSlidesToDist(var.vSlideHighBasket,VSlideTempVelo)
                            ),
                            new SleepAction(3),
                            outtake.OuttakeOut(),
                            new SleepAction(sleepTime),
                            new ParallelAction(
                                    sample1.build(),
                                    vslides.VSlidesToDist(0, VSlideTempVelo),
                                    outtake.OuttakeIdle()
                            ),
                            new SleepAction(sleepTime),

                            new SequentialAction(
                                    new ParallelAction(
                                            hslide.HSlideToDist(hSlideGrabExtension),
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
                                    vslides.VSlidesToDist(var.vSlideHighBasket,VSlideTempVelo)
                            ),
                            new SleepAction(3),
                            outtake.OuttakeOut(),
                            new SleepAction(sleepTime),
                            new ParallelAction(
                                    sample2.build(),
                                    vslides.VSlidesToDist(0, VSlideTempVelo),
                                    outtake.OuttakeIdle()
                            ),
                            new SleepAction(sleepTime),

                            new SequentialAction(
                                    new ParallelAction(
                                            hslide.HSlideToDist(hSlideGrabExtension),
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
                                    vslides.VSlidesToDist(var.vSlideHighBasket,VSlideTempVelo)
                            ),
                            new SleepAction(3),
                            outtake.OuttakeOut(),
                            new SleepAction(sleepTime),
                            new ParallelAction(
                                    park.build(),
                                    vslides.VSlidesToDist(650,VSlideTempVelo),
                                    outtake.OuttakeIdle()
                            )

                    )
            );
        }
    }
