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

import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;


@TeleOp(name = "BraulioAutoRunToPosHslideTest", group = "Autonomous")
public class BraulioAutoRunToPosHslideTest extends LinearOpMode {
    public int distance = 0;
    public class HorizontalSlides {
        private DcMotor hSlides;

        public HorizontalSlides(HardwareMap hardwareMap) {
            hSlides = hardwareMap.get(DcMotor.class, "hSlides");
            hSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hSlides.setDirection(DcMotor.Direction.FORWARD);
        }
        public class hSlideToMax implements Action  {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hSlides.setTargetPosition(800);
                hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hSlides.setPower(0.5);
                while(hSlides.isBusy()) {
                    telemetry.addData("H SLIDES BUSY", 800);
                    telemetry.update();
                }
                return false;
            }
        }
        public Action HSlideToMax() {
                return new hSlideToMax();
        }
        public class hSlideTo0 implements Action  {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hSlides.setTargetPosition(0);
                hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hSlides.setPower(0.5);
                while(hSlides.isBusy()) {
                    telemetry.addData("H SLIDES BUSY", 0);
                    telemetry.update();
                }
                return false;
            }
        }
        public Action HSlideTo0() {
            return new hSlideTo0();
        }
    }
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        HorizontalSlides hslide = new HorizontalSlides(hardwareMap);
        waitForStart();

        TrajectoryActionBuilder t1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, -20))
                .waitSeconds(1);
        TrajectoryActionBuilder twait = drive.actionBuilder(beginPose)
                .waitSeconds(1);

        Action wait = twait.build();

        if (isStopRequested()) {
            return;
        }
            Actions.runBlocking(
                    new SequentialAction(
                            t1.build(),
                            hslide.HSlideToMax(),
                            hslide.HSlideTo0()
                    )
            );
        }
    }
