package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.PRE_LM1_TESTING.Autonomous.SubsystemsPaths;

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
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM2_SUBSYSTEMS.OuttakeLM2;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Specigrabber;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Wrist;

@Disabled

@Autonomous(name = "Auto ALL Subsystems Test", group = "Autonomous")
public class BraulioAuto_ALL_SubsystemsTest extends LinearOpMode {
    public int distance = 0;

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(-90));

        // INSTANTIATE SUBSYSTEMS AND DT

        // I made all the subsystem classes check them out to understand how custom actions work!!!!

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        HorizontalSlides hslide = new HorizontalSlides(hardwareMap);

        VerticalSlides vslides = new VerticalSlides(hardwareMap);

        Elbow elbow = new Elbow(hardwareMap);

        Wrist wrist = new Wrist(hardwareMap);

        Hand hand = new Hand(hardwareMap);

        Lights lights = new Lights(hardwareMap);

        OuttakeLM2 outtakeLM2 = new OuttakeLM2(hardwareMap);

        Specigrabber specigrabber = new Specigrabber(hardwareMap);

        Actions.runBlocking(

                // MAKE SERVOS STAY IN PLACE AND STUFF

                new ParallelAction(
                        outtakeLM2.OuttakeIdle(),
                        specigrabber.SpecigrabberClose(),
                        elbow.ElbowMiddle(),
                        wrist.WristMiddle(),
                        lights.LightsRainbow(),
                        hand.HandStop()
                )
        );

        waitForStart();

        TrajectoryActionBuilder t1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, -20))
                .waitSeconds(1);
        TrajectoryActionBuilder t2 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, 20))
                .waitSeconds(1);

        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    new SequentialAction(

                            /* This code uses a combination of different actions
                            to showcase what is possible and so we can better understand
                             how this works */

                            t1.build(),
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
                                    hslide.HSlideToTransfer(),
                                    wrist.WristTransfer(),
                                    elbow.ElbowTransfer(),
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
                                    vslides.VSlidesToDistPIDF(1500),
                                    lights.LightsRed()
                            ),
                            t2.build(),
                            new ParallelAction(
                                    outtakeLM2.OuttakeOut()
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    outtakeLM2.OuttakeIdle()
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    vslides.VSlidesTo0(),
                                    lights.LightsRainbow()
                            ),
                            specigrabber.SpecigrabberOpen()
                    )
            );
        }
    }
