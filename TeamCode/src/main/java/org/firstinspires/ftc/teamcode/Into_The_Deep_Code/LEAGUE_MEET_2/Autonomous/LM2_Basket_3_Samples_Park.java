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


@Autonomous(name = "LM2 - Basket 3 Samples Park", group = "Autonomous")
public class LM2_Basket_3_Samples_Park extends LinearOpMode {
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
                //trajectories go backwards
                .setReversed(true)

                //move slides to high basket pos
                .afterTime(0, vslides.VSlidesToDist(var.vSlideHighBasket, VSlideTempVelo))

                //grabber out of way
                .afterTime(0, specigrabber.SpecigrabberClose())

                //robot prepares the outtake to score faster
                .afterTime(1, outtakeLM2.OuttakeHalfOut())

                //prepare to intake sample TWO while scoring sample ONE
                .afterTime(2, hslide.HSlideToDist(hSlideGrabExtension))
                .afterTime(2, wrist.WristIntake())
                .afterTime(3, elbow.PrepElbowIntake())
                .afterTime(3, hand.HandIntake())

                /*
                This strafeTo moves to the basket, afterTime() actions happen after whatever trajectory
                comes next so all the actions before this happen "dt" seconds after this movement,
                 */
                .strafeToLinearHeading(new Vector2d(36, 61), Math.toRadians(180))
                .waitSeconds(sleepTime)

                //score sample number ONE
                .afterTime(0, outtakeLM2.OuttakeOut())
                .waitSeconds(sleepTime)

                //grab sample TWO and bring back the outtake
                .afterTime(0, elbow.ElbowIntake())
                .afterTime(0, outtakeLM2.OuttakeIdle())
                .waitSeconds(sleepTime)

                //lower slides back to successfully transfer the next sample
                .afterTime(0, vslides.VSlidesTo0())
                .waitSeconds(sleepTime)

                //prepare to transfer sample TWO to the outtake
                .afterTime(0, hand.HandStop())
                .afterTime(0, wrist.WristMiddle())
                .afterTime(0, elbow.ElbowMiddle())
                .afterTime(0, hslide.HSlideToTransfer())
                .waitSeconds(sleepTime*3)

                //transfer sample TWO
                .afterTime(0, wrist.WristTransfer())
                .afterTime(0, elbow.ElbowTransfer())
                //this is only to be safe, if the wrist and elbow can
                //be in transfer position while the slides come down
                //just do that instead
                .afterTime(0.75, hand.HandOuttake())
                .waitSeconds(sleepTime)

                //prepare to intake sample THREE while scoring sample TWO
                .afterTime(0, hand.HandStop())
                .afterTime(0, hslide.HSlideToDist(hSlideGrabExtension))
                .afterTime(0, wrist.WristIntake())
                .afterTime(0, elbow.ElbowMiddle())
                .afterTime(0, vslides.VSlidesToDist(var.vSlideHighBasket, VSlideTempVelo))
                .waitSeconds(sleepTime*3)

                //score sample TWO
                .afterTime(0, outtakeLM2.OuttakeOut())
                .waitSeconds(sleepTime)

                //reset outtake after scoring sample 2
                .afterTime(0, outtakeLM2.OuttakeIdle())
                .waitSeconds(sleepTime)

                //lower slides to transfer sample 3
                .afterTime(0, vslides.VSlidesToDist(0, VSlideTempVelo))

                //move to next scoring position
                .strafeToLinearHeading(new Vector2d(59, 53.5), Math.toRadians(247))

                //prepare to transfer sample 3
                .afterTime(0, elbow.PrepElbowIntake())
                .afterTime(0, hand.HandIntake())
                .waitSeconds(sleepTime)

                //grab sample 3
                .afterTime(0, elbow.ElbowIntake())
                .waitSeconds(sleepTime)

                //prepare for transfer to basket
                .afterTime(0, hand.HandStop())
                .afterTime(0, wrist.WristTransfer())
                .afterTime(0, elbow.ElbowTransfer())
                .afterTime(0, hslide.HSlideToTransfer())
                .waitSeconds(sleepTime)

                //score sample 3
                .afterTime(0, hand.HandOuttake())
                .turn(Math.toRadians(23))
                .waitSeconds(10)

                //prepare slides for scoring sample 3
                .afterTime(0, hand.HandStop())
                .afterTime(0, wrist.WristIntake())
                .afterTime(0, elbow.ElbowMiddle())
                .afterTime(0, vslides.VSlidesToDist(var.vSlideHighBasket, VSlideTempVelo))
                .waitSeconds(3)

                //score sample 3
                .afterTime(0, outtakeLM2.OuttakeOut())
                .waitSeconds(sleepTime)

                //reset outtake and lower slides
                .afterTime(0, outtakeLM2.OuttakeIdle())
                .waitSeconds(sleepTime)
                .afterTime(0, vslides.VSlidesToDist(1030, VSlideTempVelo))
                .turn(Math.toRadians(-23))
                .waitSeconds(6)

                //park in the designated area
                .strafeToLinearHeading(new Vector2d(40,12), Math.toRadians(0))
                .strafeTo(new Vector2d(21,12));


        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    path.build()
            );
        }
    }
