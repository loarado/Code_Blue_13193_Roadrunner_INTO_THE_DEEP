package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    //Instantiate the sensor/servo/motor
    public Servo wrist;

    // Import final variables
    SubsystemsVariables var = new SubsystemsVariables();


    public Wrist(HardwareMap hardwareMap) {

        //Constructor
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.FORWARD);

    }

    //Actions are below
    
    public class wristOuttake implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            wrist.setPosition(var.OuttakeWristPos);

            return false;
        }
    }
    public Action WristOuttake() {
        return new wristOuttake();
    }



    public class wristMiddle implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            wrist.setPosition(var.MiddleWristPos);

            return false;
        }
    }
    public Action WristMiddle() {
        return new wristMiddle();
    }



    public class wristIntake implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            wrist.setPosition(var.IntakeWristPos);

            return false;
        }
    }
    public Action WristIntake() {
        return new wristIntake();
    }



    public class wristDebug implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            wrist.setPosition(var.DebugWristPos);

            return false;
        }
    }
    public Action WristDebug() {
        return new wristDebug();
    }



    public class wristToDist implements Action  {

        double distance = 0;

        public wristToDist(double dist) {
            //Constructor to set "distance"
            distance = dist;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            wrist.setPosition(distance);

            return false;
        }
    }
    public Action WristToDist(double dist) {
        return new wristToDist(dist);
    }


    //ADD MORE ACTIONS HERE IF NEEDED


    }