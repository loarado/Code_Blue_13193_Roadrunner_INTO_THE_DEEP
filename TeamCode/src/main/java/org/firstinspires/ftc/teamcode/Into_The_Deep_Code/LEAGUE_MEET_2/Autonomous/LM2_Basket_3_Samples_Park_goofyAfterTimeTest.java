package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.LEAGUE_MEET_2.Autonomous;

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
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Elbow;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Hand;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Lights;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM2_SUBSYSTEMS.OuttakeLM2;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Specigrabber;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Wrist;


@Autonomous(name = "LM2 - Basket 3 Samples Park silly AT TEST", group = "Autonomous")
public class LM2_Basket_3_Samples_Park_goofyAfterTimeTest extends LinearOpMode {
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

        OuttakeLM2 outtakeLM2 = new OuttakeLM2(hardwareMap);

        Specigrabber specigrabber = new Specigrabber(hardwareMap);

        Actions.runBlocking(

                // MAKE SERVOS STAY IN PLACE AND STUFF

                new ParallelAction(
                        outtakeLM2.OuttakeIdle(),
                        specigrabber.SpecigrabberOpen(),
                        elbow.ElbowMiddle(),
                        wrist.WristMiddle(),
                        lights.LightsBlue(),
                        hand.HandStop(),
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
                                vslides.VSlidesToDist(var.vSlideHighBasket, VSlideTempVelo),
                                specigrabber.SpecigrabberClose()
                        )
                )
                .strafeToLinearHeading(new Vector2d(36, 61), Math.toRadians(180))
                .afterTime(1, new SequentialAction(
                        new ParallelAction(hslide.HSlideToDist(hSlideGrabExtension), wrist.WristIntake()),
                        new SleepAction(sleepTime),
                        new ParallelAction(elbow.PrepElbowIntake(), hand.HandIntake()),
                        new SleepAction(2),
                        outtakeLM2.OuttakeOut(),
                        new SleepAction(sleepTime),
                        new ParallelAction(elbow.ElbowIntake(), outtakeLM2.OuttakeIdle()),
                        new SleepAction(sleepTime),
                        new ParallelAction(
                                vslides.VSlidesToDist(0, VSlideTempVelo),
                                hand.HandStop(),
                                wrist.WristMiddle(),
                                elbow.ElbowMiddle(),
                                hslide.HSlideToTransfer()
                        ),
                        new SleepAction(3),
                        new ParallelAction(wrist.WristIntake(), elbow.ElbowIntake(), hand.HandOuttake()),
                        new SleepAction(sleepTime),
                        new ParallelAction(
                                hand.HandStop(),
                                hslide.HSlideToDist(hSlideGrabExtension),
                                wrist.WristIntake(),
                                elbow.ElbowMiddle(),
                                vslides.VSlidesToDist(var.vSlideHighBasket, VSlideTempVelo)
                        ),
                        new SleepAction(3),
                        outtakeLM2.OuttakeOut(),
                        new SleepAction(sleepTime),
                        outtakeLM2.OuttakeIdle(),
                        new SleepAction(sleepTime),
                        vslides.VSlidesToDist(0, VSlideTempVelo)
                ))
                .strafeToLinearHeading(new Vector2d(59, 53.5), Math.toRadians(247))
                .waitSeconds(10)
                .afterTime(2, new SequentialAction(
                        new ParallelAction(elbow.PrepElbowIntake(), hand.HandIntake()),
                        new SleepAction(sleepTime),
                        elbow.ElbowIntake(),
                        new SleepAction(sleepTime),
                        new ParallelAction(
                                hand.HandStop(),
                                wrist.WristIntake(),
                                elbow.ElbowIntake(),
                                hslide.HSlideToTransfer()
                        ),
                        new SleepAction(sleepTime),
                        hand.HandOuttake()
                ))
                .turn(Math.toRadians(23))
                .waitSeconds(10)
                .afterTime(1, new SequentialAction(
                        new ParallelAction(
                                hand.HandStop(),
                                wrist.WristIntake(),
                                elbow.ElbowMiddle(),
                                vslides.VSlidesToDist(var.vSlideHighBasket, VSlideTempVelo)
                        ),
                        new SleepAction(3),
                        outtakeLM2.OuttakeOut(),
                        new SleepAction(sleepTime),
                        outtakeLM2.OuttakeIdle(),
                        new SleepAction(sleepTime),
                        vslides.VSlidesToDist(1030, VSlideTempVelo)))
                .turn(Math.toRadians(-23))
                .waitSeconds(6)
                .strafeToLinearHeading(new Vector2d(40,12), Math.toRadians(0))
                .strafeTo(new Vector2d(21,12));


        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    path.build()
            );
        }
    }