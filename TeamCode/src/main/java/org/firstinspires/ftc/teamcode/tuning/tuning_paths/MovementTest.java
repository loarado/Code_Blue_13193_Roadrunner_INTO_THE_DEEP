package org.firstinspires.ftc.teamcode.tuning.tuning_paths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;


@TeleOp(name = "Movement Test", group = "Autonomous")
public class MovementTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(35, 60, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        ElapsedTime time = new ElapsedTime();

        while(true) {

            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(35, 30.7))
                            .waitSeconds(1)
                            .turnTo(Math.toRadians(0))
                            .waitSeconds(1)
                            .build());

            if(time.time() > 1.5) {

                time.reset();

                beginPose = new Pose2d(35, 30.7, Math.toRadians(0));

                Actions.runBlocking(
                        drive.actionBuilder(beginPose)
                                .strafeTo(new Vector2d(35, 60))
                                .waitSeconds(1)
                                .turnTo(Math.toRadians(-90))
                                .waitSeconds(1)
                                .build());
                beginPose = new Pose2d(11.8, 62, Math.toRadians(90));
            }
        }
    }
}