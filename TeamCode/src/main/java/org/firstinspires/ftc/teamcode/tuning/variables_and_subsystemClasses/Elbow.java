package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Elbow {

    //Instantiate the sensor/servo/motor
    public Servo elbow;

    // Import final variables
    SubsystemsVariables var = new SubsystemsVariables();


    public Elbow(HardwareMap hardwareMap) {

        //Constructor
        elbow = hardwareMap.get(Servo.class, "elbow");
        elbow.setDirection(Servo.Direction.FORWARD);

    }

    // Actions are below

    public class elbowOuttake implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            elbow.setPosition(var.OuttakeElbowPos);

            return false;

        }
    }
    public Action ElbowOuttake() {
        return new elbowOuttake();
    }



    public class elbowMiddle implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            elbow.setPosition(var.MiddleElbowPos);

            return false;

        }
    }
    public Action ElbowMiddle() {
        return new elbowMiddle();
    }



    public class elbowIntake implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            elbow.setPosition(var.IntakeElbowPos);

            return false;

        }
    }
    public Action ElbowIntake() {
        return new elbowIntake();
    }



    public class elbowToDist implements Action  {

        double distance = 0;

        public elbowToDist(double dist) {
            //Constructor to set "distance"
            distance = dist;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            elbow.setPosition(distance);

            return false;

        }
    }
    public Action ElbowToDist(int dist) {
        return new elbowToDist(dist);
    }


    //ADD MORE ACTIONS HERE IF NEEDED


    }