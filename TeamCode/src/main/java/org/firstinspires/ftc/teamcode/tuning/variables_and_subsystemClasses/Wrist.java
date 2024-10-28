package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    public Servo wrist;

    SubsystemsVariables var = new SubsystemsVariables();
    
    public Wrist(HardwareMap hardwareMap) {
    }

    public void setwrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
    }
    
    public class wristOuttake implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            wrist.setPosition(var.OuttakeWristPos);

            return false;
        }
    }
    public Action wristOuttake() {
        return new wristOuttake();
    }
    public class wristMiddle implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            wrist.setPosition(var.MiddleWristPos);

            return false;
        }
    }
    public Action wristMiddle() {
        return new wristMiddle();
    }
    public class wristIntake implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            wrist.setPosition(var.IntakeWristPos);

            return false;
        }
    }
    public Action wristIntake() {
        return new wristIntake();
    }
    public class wristToDist implements Action  {
        double distance = 0;
        public wristToDist(double dist) {
            distance = dist;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            wrist.setPosition(distance);

            return false;
        }
    }
    public Action wristToDist(int dist) {
        return new wristToDist(dist);
    }
    }