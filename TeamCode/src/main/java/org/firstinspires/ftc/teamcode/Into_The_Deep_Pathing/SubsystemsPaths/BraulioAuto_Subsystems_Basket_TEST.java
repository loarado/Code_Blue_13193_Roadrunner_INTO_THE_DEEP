package org.firstinspires.ftc.teamcode.Into_The_Deep_Pathing.SubsystemsPaths;

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


@Autonomous(name = "Turn Test", group = "Autonomous")
public class BraulioAuto_Subsystems_Basket_TEST extends LinearOpMode {
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

        TrajectoryActionBuilder turn0 = drive.actionBuilder(beginPose)
                .turn(Math.toRadians(1));

        TrajectoryActionBuilder turn45 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(45));

        TrajectoryActionBuilder turn90 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(90));
        TrajectoryActionBuilder turn135 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(135));

        TrajectoryActionBuilder turn180 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(179));

        TrajectoryActionBuilder turn225 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(225));
        TrajectoryActionBuilder turn270 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(270));

        TrajectoryActionBuilder turn315 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(315));

        TrajectoryActionBuilder turn360 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(360));


        double sleepTime = 1.25;

        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    new SequentialAction(
                            turn0.build(),
                            new SleepAction(sleepTime),
                            turn45.build(),
                            new SleepAction(sleepTime),
                            turn90.build(),
                            new SleepAction(sleepTime),
                            turn135.build(),
                            new SleepAction(sleepTime),
                            turn180.build(),
                            new SleepAction(sleepTime),
                            turn225.build(),
                            new SleepAction(sleepTime),
                            turn270.build(),
                            new SleepAction(sleepTime),
                            turn315.build(),
                            new SleepAction(sleepTime),
                            turn360.build(),
                            new SleepAction(sleepTime)

                            )
                    );
        }
    }
