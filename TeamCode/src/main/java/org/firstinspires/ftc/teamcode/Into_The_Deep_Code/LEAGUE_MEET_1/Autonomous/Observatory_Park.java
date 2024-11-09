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
@Autonomous(name = "Observatory Park?", group = "Autonomous")
public class Observatory_Park extends LinearOpMode {
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

        TrajectoryActionBuilder park = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(12, 64.25));

        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    new SequentialAction(
                            park.build(),
                            specigrabber.SpecigrabberClose(),
                            lights.LightsRed(),
                            new SleepAction(2),
                            lights.LightsYellow(),
                            new SleepAction(2),
                            lights.LightsBlue(),
                            new SleepAction(2),
                            lights.LightsRainbow()

                    )
            );
        }
    }