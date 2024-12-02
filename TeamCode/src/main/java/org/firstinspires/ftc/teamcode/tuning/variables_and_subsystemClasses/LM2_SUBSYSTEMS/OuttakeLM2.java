package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM2_SUBSYSTEMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;

public class OuttakeLM2 {

    //Instantiate the sensor/servo/motor
    public Servo outtakeLeft;
    public Servo outtakeRight;

    // Import final variables
    SubsystemsVariables var = new SubsystemsVariables();


    public OuttakeLM2(HardwareMap hardwareMap) {

        //Constructor
        outtakeLeft = hardwareMap.get(Servo.class, "outtakeLeft");
        outtakeLeft.setDirection(Servo.Direction.FORWARD);
        outtakeRight = hardwareMap.get(Servo.class, "outtakeRight");
        outtakeRight.setDirection(Servo.Direction.FORWARD);

    }


    // Actions are below

    public class outtakeOut implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outtakeLeft.setPosition(var.outtakeLeftOut);
            outtakeRight.setPosition(var.outtakeRightOut);

            return false;

        }
    }
    public Action OuttakeOut() {
        return new outtakeOut();
    }



    public class outtakeHalfOut implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outtakeLeft.setPosition(var.outtakeLeftHalfOut);
            outtakeRight.setPosition(var.outtakeRightHalfOut);

            return false;

        }
    }
    public Action OuttakeHalfOut() {
        return new outtakeHalfOut();
    }



    public class outtakeIdle implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outtakeLeft.setPosition(var.outtakeLeftIdle);
            outtakeRight.setPosition(var.outtakeRightIdle);

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

            outtakeLeft.setPosition(distance);
            outtakeRight.setPosition(distance);

            return false;

        }
    }
    public Action OuttakeToDist(double dist) {
        return new outtakeToDist(dist);
    }

    
    //ADD MORE ACTIONS HERE IF NEEDED
    

    }