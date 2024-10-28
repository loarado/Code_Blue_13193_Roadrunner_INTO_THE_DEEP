package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Specigrabber {

    //Instantiate the sensor/servo/motor
    public Servo specigrabber;

    // Import final variables
    SubsystemsVariables var = new SubsystemsVariables();


    public Specigrabber(HardwareMap hardwareMap) {

        //Constructor
        specigrabber = hardwareMap.get(Servo.class, "specigrabber");
        specigrabber.setDirection(Servo.Direction.FORWARD);

    }

    // Actions are below

    public class specigrabberClose implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            specigrabber.setPosition(var.specigrabberClosed);

            return false;

        }
    }
    public Action SpecigrabberClose() {
        return new specigrabberClose();
    }



    public class specigrabberOpen implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            specigrabber.setPosition(var.specigrabberOpen);

            return false;

        }
    }
    public Action SpecigrabberOpen() {
        return new specigrabberOpen();
    }



    public class specigrabberToDist implements Action  {

        double distance = 0;

        public specigrabberToDist(double dist) {
            //Constructor to set "distance"
            distance = dist;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            specigrabber.setPosition(distance);

            return false;

        }
    }
    public Action SpecigrabberToDist(int dist) {
        return new specigrabberToDist(dist);
    }

    
    //ADD MORE ACTIONS HERE IF NEEDED
    

    }