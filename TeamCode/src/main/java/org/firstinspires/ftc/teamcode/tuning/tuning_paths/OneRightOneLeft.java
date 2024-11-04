package org.firstinspires.ftc.teamcode.tuning.tuning_paths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;


@Autonomous(name = "One right One left", group = "Autonomous")
public class OneRightOneLeft extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        ElapsedTime time = new ElapsedTime();

        while(true) {

            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose).strafeTo(new Vector2d(36.3, 61.7)).waitSeconds(2).build());

            if(time.time() > 1) {

                time.reset();

                beginPose = new Pose2d(36.3, 61.7, Math.toRadians(90));

                Actions.runBlocking(
                        drive.actionBuilder(beginPose).strafeTo(new Vector2d(11.8, 61.7)).waitSeconds(2).build());

                beginPose = new Pose2d(11.8, 61.7, Math.toRadians(90));

            }
        }
    }
}