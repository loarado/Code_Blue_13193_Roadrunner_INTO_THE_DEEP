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


@Autonomous(name = "Basket 2 Samples Park", group = "Autonomous")
public class Basket_2_Samples_Park extends LinearOpMode {
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

        Pose2d bucketEnd = new Pose2d(56, 56, Math.toRadians(225));

        TrajectoryActionBuilder sample1 = drive.actionBuilder(bucketEnd)
                .setReversed(false)
                .strafeTo(new Vector2d(48, 48))
                .turn(Math.toRadians(48));

        Pose2d sampleEnd = new Pose2d(48, 48, Math.toRadians(272));

        TrajectoryActionBuilder bucket2 = drive.actionBuilder(sampleEnd)
                .turn(Math.toRadians(-48))
                .strafeTo(new Vector2d(56, 56));

        TrajectoryActionBuilder park = drive.actionBuilder(bucketEnd)
                .strafeToLinearHeading(new Vector2d(40,12), Math.toRadians(0))
                .strafeTo(new Vector2d(21,12));

        double sleepTime = 1.5;

        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    new SequentialAction(

                            new ParallelAction(
                                    bucket1.build(),
                                    vslides.VSlidesToDist(var.vSlideHighBasket,VSlideTempVelo)
                            ),
                            specigrabber.SpecigrabberClose(),
                            new SleepAction(2),
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
                                            hand.HandIntake()
                                    ),
                                    new SleepAction(1),
                                    elbow.PrepElbowIntake(),
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
                            new SleepAction(2),
                            outtake.OuttakeOut(),
                            new SleepAction(sleepTime),
                            new ParallelAction(
                                    park.build(),
                                    vslides.VSlidesToDist(1030,VSlideTempVelo),
                                    outtake.OuttakeIdle()
                            )

                    )
            );
        }
    }
