package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lights {
    public RevBlinkinLedDriver lights;

    SubsystemsVariables var = new SubsystemsVariables();
    
    public Lights(HardwareMap hardwareMap) {
    }

    public void setlights(HardwareMap hardwareMap) {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
    }
    
    public class lightsBlue implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

            return false;
        }
    }
    public Action LightsBlue() {
        return new lightsBlue();
    }
    public class lightsRed implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            return false;
        }
    }
    public Action LightsRed() {
        return new lightsRed();
    }
    public class lightsYellow implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

            return false;
        }
    }
    public Action LightsYellow() {
        return new lightsYellow();
    }
    }