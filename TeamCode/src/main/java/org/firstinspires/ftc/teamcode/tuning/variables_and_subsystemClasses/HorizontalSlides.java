package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizontalSlides {
    public DcMotor hSlides;

    SubsystemsVariables var = new SubsystemsVariables();

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
        hSlides.setTargetPosition(var.hSlideRuleMax);
        hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hSlides.setPower(var.hSlideSpeed);
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
            if(dist<=800) {
                distance = dist;
            }else{
                distance = 800;
            }
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            hSlides.setTargetPosition(distance);
            hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hSlides.setPower(var.hSlideSpeed);
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
        hSlides.setPower(var.hSlideSpeed);
        while(hSlides.isBusy()) {
            
        }
        return false;
    }
}
public Action HSlideTo0() {
    return new hSlideTo0();
}
    }