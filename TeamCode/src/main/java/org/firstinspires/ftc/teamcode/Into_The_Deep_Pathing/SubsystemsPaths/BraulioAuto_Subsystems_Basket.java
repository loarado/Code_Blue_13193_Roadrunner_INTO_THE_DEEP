package org.firstinspires.ftc.teamcode.Into_The_Deep_Pathing.SubsystemsPaths;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Elbow;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Hand;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Lights;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Outtake;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Specigrabber;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Wrist;


@Autonomous(name = "Auto Basket Test", group = "Autonomous")
public class BraulioAuto_Subsystems_Basket extends LinearOpMode {
    public int distance = 0;

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(36, 64.25, Math.toRadians(-90));

        // INSTANTIATE SUBSYSTEMS AND DT

        // I made all the subsystem classes check them out to understand how custom actions work!!!!

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
                        lights.LightsRainbow(),
                        hand.HandStop()
                )
        );

        waitForStart();

        TrajectoryActionBuilder offTheWall = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(48, 40))
                .waitSeconds(2)
                .turn(Math.toRadians(-45))
                .strafeTo(new Vector2d(54,54))
                .waitSeconds(2)
                .strafeTo(new Vector2d(54, 50))
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(58, 40))
                .strafeTo(new Vector2d(54,40))
                .turn(Math.toRadians(-45))
                .strafeTo(new Vector2d(54,54))
                .waitSeconds(2)
                .strafeTo(new Vector2d(54, 50))
                .turn(Math.toRadians(135))
                .strafeTo(new Vector2d(58,28))
                .waitSeconds(2)
                .strafeTo(new Vector2d(50,28))
                .turn(Math.toRadians(225))
                .strafeTo(new Vector2d(54,54))
                .waitSeconds(2)
                .splineTo(new Vector2d(40,12), Math.toRadians(180))
                .strafeTo(new Vector2d(24,12))
                .waitSeconds(2)
                .build();
        TrajectoryActionBuilder t2 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, 20))
                .waitSeconds(1);

        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    new SequentialAction(

                            /* This code uses a combination of different actions
                            to showcase what is possible and so we can better understand
                             how this works */

                            offTheWall.build(),
                            hslide.HSlideToMax(),
                            new ParallelAction(
                              hand.HandIntake(),
                              elbow.ElbowIntake(),
                              lights.LightsYellow()
                            ),
                            new SleepAction(1),
                            wrist.WristIntake(),
                            new SleepAction(1),
                            new ParallelAction(
                                    wrist.WristMiddle(),
                                    elbow.ElbowMiddle(),
                                    hand.HandStop()
                            ),
                            new ParallelAction(
                                    hslide.HSlideTo0(),
                                    wrist.WristOuttake(),
                                    elbow.ElbowOuttake(),
                                    lights.LightsBlue()
                            ),
                            hand.HandOuttake(),
                            new SleepAction(1),
                            new ParallelAction(
                                    wrist.WristMiddle(),
                                    elbow.ElbowMiddle(),
                                    hslide.HSlideToMax(),
                                    hand.HandStop()
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    wrist.WristIntake(),
                                    vslides.VSlidesToDist(1500),
                                    lights.LightsRed()
                            ),
                            t2.build(),
                            new ParallelAction(
                                    outtake.OuttakeOut(),
                                    specigrabber.SpecigrabberOpen()
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtake.OuttakeIdle(),
                                    specigrabber.SpecigrabberClose()
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    vslides.VSlidesTo0(),
                                    lights.LightsRainbow(),
                                    specigrabber.SpecigrabberOpen()
                            )
                    )
            );
        }
    }
