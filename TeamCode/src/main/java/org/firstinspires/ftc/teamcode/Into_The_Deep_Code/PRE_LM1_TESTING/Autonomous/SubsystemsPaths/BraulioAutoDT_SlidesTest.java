package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.PRE_LM1_TESTING.Autonomous.SubsystemsPaths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Lights;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;

@Disabled

@Autonomous(name = "Auto DT & Slides Test", group = "Autonomous")
public class BraulioAutoDT_SlidesTest extends LinearOpMode {
    public int distance = 0;

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(-90));

        // INSTANTIATE SUBSYSTEMS AND DT

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        HorizontalSlides hslide = new HorizontalSlides(hardwareMap);

        VerticalSlides vslides = new VerticalSlides(hardwareMap);

        Lights lights = new Lights(hardwareMap);

        //END OF INITIALIZATION

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
                            t1.build(),
                            lights.LightsBlue(),
                            hslide.HSlideToMax(),
                            lights.LightsRed(),
                            hslide.HSlideTo0(),
                            hslide.HSlideToDist(400),
                            vslides.VSlidesToDist(1000),
                            hslide.HSlideTo0(),
                            lights.LightsYellow(),
                            vslides.VSlidesToDist(2000),
                            hslide.HSlideToDist(100),
                            hslide.HSlideToDist(700),
                            vslides.VSlidesTo0(),
                            lights.LightsRainbow(),
                            t2.build()
                    )
            );
        }
    }
