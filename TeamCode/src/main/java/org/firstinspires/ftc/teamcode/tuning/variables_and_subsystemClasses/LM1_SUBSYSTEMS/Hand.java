package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;

public class Hand {

    //Instantiate the sensor/servo/motor
    public CRServo leftHand;
    public CRServo rightHand;

    // Import final variables
    SubsystemsVariables var = new SubsystemsVariables();


    public Hand(HardwareMap hardwareMap) {

        //Constructor
        leftHand = hardwareMap.get(CRServo.class, "leftHand");
        leftHand.setDirection(CRServo.Direction.FORWARD);
        rightHand = hardwareMap.get(CRServo.class, "rightHand");
        rightHand.setDirection(CRServo.Direction.FORWARD);

    }

    // Actions are below
    
    public class handOuttake implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            leftHand.setPower(var.leftHandOut);
            rightHand.setPower(var.rightHandOut);

            return false;
        }
    }
    public Action HandOuttake() {
        return new handOuttake();
    }



    public class handStop implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            leftHand.setPower(var.leftHandStop);
            rightHand.setPower(var.rightHandStop);

            return false;
        }
    }
    public Action HandStop() {
        return new handStop();
    }



    public class handIntake implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            leftHand.setPower(var.leftHandIn);
            rightHand.setPower(var.rightHandIn);

            return false;
        }
    }
    public Action HandIntake() {
        return new handIntake();
    }


    //ADD MORE ACTIONS HERE IF NEEDED


    }