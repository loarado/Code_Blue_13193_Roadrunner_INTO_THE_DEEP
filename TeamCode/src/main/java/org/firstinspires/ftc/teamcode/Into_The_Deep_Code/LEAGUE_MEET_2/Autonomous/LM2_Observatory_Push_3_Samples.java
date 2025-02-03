package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.LEAGUE_MEET_2.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Elbow;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM3_SUBSYSTEMS.HandLM3;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Lights;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM3_SUBSYSTEMS.SpecigrabberLM3;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Wrist;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM2_SUBSYSTEMS.OuttakeLM2;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;


@Autonomous(name = "Observatory Push 3 " +
        "Samples", group = "Autonomous")
public class LM2_Observatory_Push_3_Samples extends LinearOpMode {
    public int distance = 0;

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(-24, 64.25, Math.toRadians(270));

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


        TrajectoryActionBuilder path = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(-33,48, Math.toRadians(270)),Math.toRadians(270))
                .splineTo(new Vector2d(-36,12),Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-46,12,Math.toRadians(270)),Math.toRadians(90))
                .strafeTo(new Vector2d(-46,53))
                .strafeTo(new Vector2d(-46,18))
                .splineToLinearHeading(new Pose2d(-55,12,Math.toRadians(270)),Math.toRadians(90))
                .strafeTo(new Vector2d(-55,53))
                .strafeTo(new Vector2d(-55,18))
                .splineToLinearHeading(new Pose2d(-61,12,Math.toRadians(270)),Math.toRadians(90))
                .strafeTo(new Vector2d(-61,53))
                .strafeTo(new Vector2d(-61,40))
                .waitSeconds(10)
                /*delay to allow human player to get samples out of observation zone before parking*/
                .strafeTo(new Vector2d(-61,63));


        if (isStopRequested()) { return; }

            Actions.runBlocking(
                    path.build()
            );
        }
    }
