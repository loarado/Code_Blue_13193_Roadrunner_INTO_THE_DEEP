package org.firstinspires.ftc.teamcode.Into_The_Deep_Pathing.BasicPaths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;


@TeleOp(name = "Alex Pathing Basket", group = "Autonomous")
public class AlexPathingBasket extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(36, 64.25, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(48, 40))
                            .waitSeconds(2)
                            .turn(Math.toRadians(-45))
                            .strafeTo(new Vector2d(54,54))
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(54, 50))
                            .turn(Math.toRadians(45))
                            .strafeTo(new Vector2d(58, 40))
                            .strafeTo(new Vector2d(54,40))
                            .turn(Math.toRadians(-45))
                            .strafeTo(new Vector2d(54,54))
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(54, 50))
                            .turn(Math.toRadians(135))
                            .strafeTo(new Vector2d(58,28))
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(50,28))
                            .turn(Math.toRadians(225))
                            .strafeTo(new Vector2d(54,54))
                            .waitSeconds(2)
                            .splineTo(new Vector2d(40,12), Math.toRadians(180))
                            .strafeTo(new Vector2d(24,12))
                            .waitSeconds(2)
                            .build());

    }
}