package org.firstinspires.ftc.teamcode.Into_The_Deep_Pathing.SubsystemsPaths;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.HorizontalSlides;


@Autonomous(name = "BraulioAutoRunToPosHslideVARTest", group = "Autonomous")
public class BraulioAutoRunToPosHslideVARTest extends LinearOpMode {
    public int distance = 0;

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        HorizontalSlides hslide = new HorizontalSlides(hardwareMap);
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
                            hslide.HSlideTo0(),
                            hslide.HSlideToDist(100),
                            hslide.HSlideToDist(700)

                    )
            );
        }
    }
