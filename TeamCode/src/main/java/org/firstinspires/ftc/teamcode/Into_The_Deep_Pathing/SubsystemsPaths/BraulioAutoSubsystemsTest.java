package org.firstinspires.ftc.teamcode.Into_The_Deep_Pathing.SubsystemsPaths;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Elbow;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Hand;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Wrist;


@TeleOp(name = "BraulioAutoSubsystemsTest", group = "Autonomous")
public class BraulioAutoSubsystemsTest extends LinearOpMode {
    public int distance = 0;

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        HorizontalSlides hslide = new HorizontalSlides(hardwareMap);
        hslide.sethSlides(hardwareMap);

        VerticalSlides vslides = new VerticalSlides(hardwareMap);
        vslides.setVSlides(hardwareMap);

        Elbow elbow = new Elbow(hardwareMap);
        elbow.setelbow(hardwareMap);

        Wrist wrist = new Wrist(hardwareMap);
        wrist.setwrist(hardwareMap);

        Hand hand = new Hand(hardwareMap);
        hand.setHands(hardwareMap);

        waitForStart();

        TrajectoryActionBuilder t1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, -20))
                .waitSeconds(1);
        TrajectoryActionBuilder twait = drive.actionBuilder(beginPose)
                .waitSeconds(1);

        Action wait = twait.build();

        if (isStopRequested()) {
            return;
        }

            Actions.runBlocking(
                    new SequentialAction(
                            t1.build(),
                            hslide.HSlideToMax(),
                            hslide.HSlideTo0(),
                            hslide.HSlideToDist(400),
                            vslides.VSlidesToDist(1000),
                            hslide.HSlideTo0(),
                            vslides.VSlidesToDist(2000),
                            hslide.HSlideToDist(100),
                            hslide.HSlideToDist(700),
                            vslides.VSlidesTo0()

                    )
            );
        }
    }
