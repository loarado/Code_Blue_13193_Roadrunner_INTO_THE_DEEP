package org.firstinspires.ftc.teamcode.Into_The_Deep_Pathing.BasicPaths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;

@Disabled
@Autonomous(name = "Nicholas Pathing Observatory", group = "Autonomous")
public class NicholasPathingObservatory extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-24, 64.25, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(-8, 34))
                        .waitSeconds(3)
                        .strafeTo(new Vector2d(-48, 40))
                        .waitSeconds(.5)
                        .strafeToSplineHeading(new Vector2d(-48, 60), Math.toRadians(90))
                        .waitSeconds(.5)
                        .strafeTo(new Vector2d(-48,12))
                        .waitSeconds(.5)
                        .strafeTo(new Vector2d(-60,12))
                        .waitSeconds(.5)
                        .strafeToSplineHeading(new Vector2d(-58, 60), Math.toRadians(180))
                        .waitSeconds(.5)
                        .strafeTo(new Vector2d(-58,24))
                        .waitSeconds(.5)
                        .strafeTo(new Vector2d(-62,24))
                        .waitSeconds(.5)
                        .strafeToSplineHeading(new Vector2d(-62,60),Math.toRadians(90))
                        .waitSeconds(.5)
                        .build());
    }
}