package org.firstinspires.ftc.teamcode.Into_The_Deep_Pathing.BasicPaths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;


@Autonomous(name = "Nicholas Pathing Basket", group = "Autonomous")
public class NicholasPathingBasket extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(35, 64.25, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(48,28 ))
                            .strafeTo(new Vector2d(48,38 ))
                            .splineTo(new Vector2d(55,55), Math.toRadians(45))
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(55,50))
                            .turn(Math.toRadians(45))
                            .splineTo(new Vector2d(58,28 ), Math.toRadians(-90))
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(58,38))
                            .splineTo(new Vector2d(55,55), Math.toRadians(45))
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(45,45))
                            .splineTo(new Vector2d(60,28), Math.toRadians(0))
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(50,30))
                            .splineTo(new Vector2d(55,55), Math.toRadians(45))
                            .waitSeconds(2)
                            .build());
    }
}