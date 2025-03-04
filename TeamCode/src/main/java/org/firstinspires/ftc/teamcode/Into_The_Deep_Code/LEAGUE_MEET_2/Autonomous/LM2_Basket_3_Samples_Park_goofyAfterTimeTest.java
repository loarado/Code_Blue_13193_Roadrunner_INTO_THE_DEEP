package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.LEAGUE_MEET_2.Autonomous;

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
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM3_SUBSYSTEMS.HandLM3;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Lights;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM2_SUBSYSTEMS.OuttakeLM2;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM3_SUBSYSTEMS.SpecigrabberLM3;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Wrist;

@Disabled
@Autonomous(name = "Basket 3 Samples Park", group = "Autonomous")
public class LM2_Basket_3_Samples_Park_goofyAfterTimeTest extends LinearOpMode {

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

        HandLM3 handLM3 = new HandLM3(hardwareMap);

        Lights lights = new Lights(hardwareMap);

        OuttakeLM2 outtakeLM2 = new OuttakeLM2(hardwareMap);

        SpecigrabberLM3 specigrabber = new SpecigrabberLM3(hardwareMap);

        Actions.runBlocking(

                // MAKE SERVOS STAY IN PLACE AND STUFF

                new ParallelAction(
                        outtakeLM2.OuttakeIdle(),
                        specigrabber.SpecigrabberOpen(),
                        elbow.ElbowMiddle(),
                        wrist.WristMiddle(),
                        lights.LightsBlue(),
                        handLM3.HandStop(),
                        hslide.HSlideTo0()
                )
        );

        waitForStart();

        int hSlideGrabExtension = var.hSlideRuleMax;

        int VSlideTempVelo = 1300;

        double sleepTime = 1;

        TrajectoryActionBuilder path = drive.actionBuilder(beginPose)
                .setReversed(true)
                .afterTime(.5,
                        new ParallelAction(
                                vslides.VSlidesToDist(2377, VSlideTempVelo),
                                specigrabber.SpecigrabberClose()
                        )
                )
                .strafeToLinearHeading(new Vector2d(36, 61), Math.toRadians(180))
                .afterTime(1, new SequentialAction(
                        new ParallelAction(hslide.HSlideToDist(hSlideGrabExtension), wrist.WristIntake()),
                        new SleepAction(sleepTime),
                        new ParallelAction(elbow.PrepElbowIntake(), handLM3.HandIntake()),
                        new SleepAction(1),
                        outtakeLM2.OuttakeOut(),
                        new SleepAction(sleepTime),
                        new ParallelAction(elbow.ElbowIntake(), outtakeLM2.OuttakeIdle(), vslides.VSlidesToDist(0, VSlideTempVelo)),
                        new SleepAction(1.25),
                        new ParallelAction(
                                handLM3.HandStop(),
                                wrist.WristTransfer(),
                                elbow.ElbowTransfer()
                        ),
                        new SleepAction(0.25),
                        hslide.HSlideToTransfer(),
                        new SleepAction(0.75),
                        new ParallelAction(handLM3.HandOuttake()),
                        new SleepAction(sleepTime),
                        new ParallelAction(
                                handLM3.HandStop(),
                                hslide.HSlideToDist(hSlideGrabExtension-30),
                                wrist.WristIntake(),
                                elbow.ElbowMiddle(),
                                vslides.VSlidesToDist(2377, VSlideTempVelo)
                        ),
                        new SleepAction(1.25),
                        outtakeLM2.OuttakeOut(),
                        new SleepAction(sleepTime),
                        new ParallelAction(outtakeLM2.OuttakeIdle(),vslides.VSlidesToDist(0, VSlideTempVelo)),
                        new SleepAction(1.25)
                ))
                .strafeToLinearHeading(new Vector2d(59, 52.5), Math.toRadians(255.5))
                .waitSeconds(12.5)
                .afterTime(1, new SequentialAction(
                        new ParallelAction(elbow.PrepElbowIntake(), handLM3.HandIntake(), outtakeLM2.OuttakeIdle()),
                        new SleepAction(0.5),
                        elbow.ElbowIntake(),
                        new SleepAction(0.75),
                        new ParallelAction(
                                handLM3.HandStop(),
                                wrist.WristTransfer(),
                                elbow.ElbowTransfer(),
                                hslide.HSlideToTransfer()
                        ),
                        new SleepAction(sleepTime),
                        handLM3.HandOuttake(),
                        new SleepAction(sleepTime)
                ))
                .turn(Math.toRadians(13.5))
                .waitSeconds(5)
                .afterTime(0, new SequentialAction(
                        new ParallelAction(
                                handLM3.HandStop(),
                                wrist.WristIntake(),
                                elbow.ElbowMiddle(),
                                vslides.VSlidesToDist(2377, VSlideTempVelo)
                        ),
                        new SleepAction(1.25),
                        outtakeLM2.OuttakeOut(),
                        new SleepAction(sleepTime),
                        outtakeLM2.OuttakeIdle(),
                        new SleepAction(sleepTime)
                        ))
                .turn(Math.toRadians(-12.5))
                .waitSeconds(5.75)
                .afterTime(0, vslides.VSlidesToDist(620, VSlideTempVelo))
                .strafeToLinearHeading(new Vector2d(40,12), Math.toRadians(0))
                .strafeTo(new Vector2d(21,12));


        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    path.build()
            );
        }
    }
