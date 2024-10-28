package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hand {
    public Servo leftHand;
    public Servo rightHand;

    SubsystemsVariables var = new SubsystemsVariables();
    
    public Hand(HardwareMap hardwareMap) {
    }

    public void setHands(HardwareMap hardwareMap) {
        leftHand = hardwareMap.get(Servo.class, "leftHand");
        leftHand.setDirection(Servo.Direction.FORWARD);
        rightHand = hardwareMap.get(Servo.class, "rightHand");
        rightHand.setDirection(Servo.Direction.FORWARD);
    }
    
    public class handOuttake implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            leftHand.setPosition(var.leftHandOut);
            rightHand.setPosition(var.rightHandOut);

            return false;
        }
    }
    public Action HandOuttake() {
        return new handOuttake();
    }
    public class handStop implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            leftHand.setPosition(var.leftHandStop);
            rightHand.setPosition(var.rightHandStop);

            return false;
        }
    }
    public Action HandStop() {
        return new handStop();
    }
    public class handIntake implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            leftHand.setPosition(var.leftHandIn);
            rightHand.setPosition(var.rightHandIn);

            return false;
        }
    }
    public Action HandIntake() {
        return new handIntake();
    }
    }