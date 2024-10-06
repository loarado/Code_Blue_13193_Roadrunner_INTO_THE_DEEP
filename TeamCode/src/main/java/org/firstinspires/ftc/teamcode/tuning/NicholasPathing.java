package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp(name = "Movement Test", group = "Autonomous")
public class NicholasPathing extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(35, 60, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(48,28 ))
                            .strafeTo(new Vector2d(48,38 ))
                            .splineTo(new Vector2d(55,55), Math.toRadians(45))
                            .strafeTo(new Vector2d(55,50))
                            .turn(Math.toRadians(45))
                            .splineTo(new Vector2d(58,28 ), Math.toRadians(-90))
                            .strafeTo(new Vector2d(58,38))
                            .splineTo(new Vector2d(55,55), Math.toRadians(45))
                            .strafeTo(new Vector2d(45,45))
                            .splineTo(new Vector2d(60,28), Math.toRadians(0))
                            .strafeTo(new Vector2d(50,30))
                            .splineTo(new Vector2d(55,55), Math.toRadians(45))
                            .build());

    }
}