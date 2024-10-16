package org.firstinspires.ftc.teamcode.sample_paths_comp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;


@TeleOp(name = "Nicholas Pathing Observatory", group = "Autonomous")
public class NicholasPathingObservatory extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-24, 64.25, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(-8,34))
                            .waitSeconds(3)
                            .strafeTo(new Vector2d(-48,40))
                            .strafeToSplineHeading(new Vector2d(-48,60,Math.toRadians(90)))
                            .build());(

    }
}