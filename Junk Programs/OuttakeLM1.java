package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;

public class OuttakeLM1 {

    //Instantiate the sensor/servo/motor
    public Servo outtake;

    // Import final variables
    SubsystemsVariables var = new SubsystemsVariables();


    public OuttakeLM1(HardwareMap hardwareMap) {

        //Constructor
        outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.FORWARD);

    }


    // Actions are below

    public class outtakeOut implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outtake.setPosition(var.outtakeLeftOut);

            return false;

        }
    }
    public Action OuttakeOut() {
        return new outtakeOut();
    }



    public class outtakeIdle implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outtake.setPosition(var.outtakeLeftIdle);

            return false;

        }
    }
    public Action OuttakeIdle() {
        return new outtakeIdle();
    }



    public class outtakeToDist implements Action  {

        double distance = 0;

        public outtakeToDist(double dist) {
            //Constructor to set "distance"
            distance = dist;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outtake.setPosition(distance);

            return false;

        }
    }
    public Action outtakeToDist(double dist) {
        return new outtakeToDist(dist);
    }

    
    //ADD MORE ACTIONS HERE IF NEEDED
    

    }