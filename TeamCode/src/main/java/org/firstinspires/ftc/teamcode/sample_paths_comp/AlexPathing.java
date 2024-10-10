package org.firstinspires.ftc.teamcode.sample_paths_comp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;


@TeleOp(name = "Alex Pathing", group = "Autonomous")
public class AlexPathing extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(36, 64.25, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(48, 28))
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(48, 38))
                            .splineTo(new Vector2d(54, 54), Math.toRadians(45))
                            .waitSeconds(2)
                            .splineTo(new Vector2d(58, 28), Math.toRadians(270))
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(52,34))
                            .splineTo(new Vector2d(54, 54), Math.toRadians(90))
                            .waitSeconds(2)
                            .splineTo(new Vector2d(58, 28), Math.toRadians(0))
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(52,34))
                            .splineTo(new Vector2d(54, 54), Math.toRadians(0))
                            .waitSeconds(2)
                            .splineTo(new Vector2d(40,12), Math.toRadians(180))
                            .strafeTo(new Vector2d(24,12))
                            .waitSeconds(2)
                            .build());

    }
}