package org.firstinspires.ftc.teamcode.tuning.tuning_paths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;


@TeleOp(name = "One Tile Backwards One Tile Forwards", group = "Autonomous")
public class OneBackOneForwards extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(11.8, 62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        ElapsedTime time = new ElapsedTime();

        while(true) {

            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose).lineToYSplineHeading(38, Math.toRadians(90)).waitSeconds(1).build());

            if(time.time() > 1.5) {

                time.reset();

                beginPose = new Pose2d(11.8, 38, Math.toRadians(90));

                Actions.runBlocking(
                        drive.actionBuilder(beginPose).lineToYSplineHeading(62, Math.toRadians(90)).waitSeconds(1).build());

                beginPose = new Pose2d(11.8, 62, Math.toRadians(90));
            }
        }
    }
}