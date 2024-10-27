package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses;

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

import org.firstinspires.ftc.teamcode.Into_The_Deep_Pathing.SubsystemsPaths.BraulioAutoRunToPosSubsystemsTest;
import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.pidTest.H_Slides_Methods;

public class HorizontalSlides {
    public DcMotor hSlides;
    public HorizontalSlides(HardwareMap hardwareMap) {
    }

    public void sethSlides(HardwareMap hardwareMap) {
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
            
        }
        return false;
    }
}
public Action HSlideToMax() {
    return new hSlideToMax();
}
    public class hSlideToDist implements Action  {
        int distance = 0;
        public hSlideToDist(int dist) {
            distance = dist;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            hSlides.setTargetPosition(distance);
            hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hSlides.setPower(0.5);
            while(hSlides.isBusy()) {

            }
            return false;
        }
    }
    public Action HSlideToDist(int dist) {
        return new hSlideToDist(dist);
    }
public class hSlideTo0 implements Action  {
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        hSlides.setTargetPosition(0);
        hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hSlides.setPower(0.5);
        while(hSlides.isBusy()) {
            
        }
        return false;
    }
}
public Action HSlideTo0() {
    return new hSlideTo0();
}
    }