package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM3_SUBSYSTEMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;

public class HandLM3 {

    //Instantiate the sensor/servo/motor
    public CachingCRServo hand;

    // Import final variables
    SubsystemsVariables var = new SubsystemsVariables();


    public HandLM3(HardwareMap hardwareMap) {

        //Constructor
        hand = new CachingCRServo(hardwareMap.get(CRServo.class, "hand"));
        hand.setDirection(CRServo.Direction.FORWARD);
    }

    // Actions are below
    
    public class handOuttake implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            hand.setPower(var.handOut);

            return false;
        }
    }
    public Action HandOuttake() {
        return new handOuttake();
    }



    public class handStop implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            hand.setPower(var.handStop);

            return false;
        }
    }
    public Action HandStop() {
        return new handStop();
    }



    public class handIntake implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            hand.setPower(var.handIn);

            return false;
        }
    }
    public Action HandIntake() {
        return new handIntake();
    }


    //ADD MORE ACTIONS HERE IF NEEDED


    }