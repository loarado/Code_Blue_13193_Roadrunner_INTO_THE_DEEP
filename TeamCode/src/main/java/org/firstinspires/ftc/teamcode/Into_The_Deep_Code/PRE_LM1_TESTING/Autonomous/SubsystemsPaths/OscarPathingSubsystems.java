package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.PRE_LM1_TESTING.Autonomous.SubsystemsPaths;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;

@Disabled

@Autonomous(name = "Oscar Pathing Subsystems", group = "Autonomous")
public class OscarPathingSubsystems extends LinearOpMode {
    public class ServoTest {
        private Servo servo0;

        public ServoTest(HardwareMap hardwareMap) {
            servo0 = hardwareMap.get(Servo.class, "servo");
        }
        public class TestServo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo0.setPosition(0);
                return false;
            }
        }
        public Action testServo() {
            return new TestServo();
        }
    }
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-24, 64.25, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        ServoTest servo = new ServoTest(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;
        while(true) {
            Actions.runBlocking(
                    new SequentialAction(
                            servo.testServo()

                    )
            );
        }
    }
}