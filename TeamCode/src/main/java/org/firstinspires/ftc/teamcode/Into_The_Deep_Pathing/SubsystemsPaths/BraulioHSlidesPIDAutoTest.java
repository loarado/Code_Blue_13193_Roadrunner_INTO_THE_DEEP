package org.firstinspires.ftc.teamcode.Into_The_Deep_Pathing.SubsystemsPaths;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tuning.pidTest.H_Slides_Methods;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;


@TeleOp(name = "BraulioHSlidesPIDAutoTest", group = "Autonomous")
public class BraulioHSlidesPIDAutoTest extends LinearOpMode {
    public double distance = 0;
    public class HorizontalSlides {
        private DcMotor hSlides;

        public HorizontalSlides(HardwareMap hardwareMap) {
            hSlides = hardwareMap.get(DcMotor.class, "hSlides");
        }
        public class hSlideTo implements Action  {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                while(!(hSlides.getCurrentPosition()>(distance-10)&&hSlides.getCurrentPosition()<(distance+10))) {
                    hSlides.setPower(H_Slides_Methods.returnPID(hSlides.getCurrentPosition(), distance));
                }
                return false;
            }
        }
        public Action HSlideTo(double dist) {
            distance = dist;
            return new hSlideTo();
        }
    }
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        HorizontalSlides hslide = new HorizontalSlides(hardwareMap);
        waitForStart();

        TrajectoryActionBuilder t1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, -2))
                .waitSeconds(1);
        TrajectoryActionBuilder twait = drive.actionBuilder(beginPose)
                .waitSeconds(1);

        Action wait = twait.build();

        if (isStopRequested()) return;

            Actions.runBlocking(
                    new SequentialAction(
                            t1.build(),
                            hslide.HSlideTo(300),
                            hslide.HSlideTo(50),
                            hslide.HSlideTo(700),
                            hslide.HSlideTo(200),
                            hslide.HSlideTo(600)

                    )
            );
        }
    }
