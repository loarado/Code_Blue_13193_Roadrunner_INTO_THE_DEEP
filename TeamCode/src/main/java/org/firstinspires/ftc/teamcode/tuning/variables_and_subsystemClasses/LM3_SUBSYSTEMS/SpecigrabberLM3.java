package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM3_SUBSYSTEMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.pidTest.SpecimenArm_PID_Class;
import org.firstinspires.ftc.teamcode.tuning.pidTest.V_Slides_PID_Class;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class SpecigrabberLM3 {

    //Instantiate the sensor/servo/motor
    public CachingServo specClawOpen;
    public CachingServo specClawRotate;
    public DcMotorEx specimenArm;
    public int setPosition;

    // Import final variables
    SubsystemsVariables var = new SubsystemsVariables();


    public SpecigrabberLM3(HardwareMap hardwareMap) {

        //Constructor
        specClawOpen = new CachingServo(hardwareMap.get(Servo.class, "specClawOpen"));
        specClawOpen.setDirection(Servo.Direction.FORWARD);
        specClawRotate = new CachingServo(hardwareMap.get(Servo.class, "specClawRotate"));
        specClawRotate.setDirection(Servo.Direction.FORWARD);
        specimenArm = hardwareMap.get(DcMotorEx.class, "specimenArm");
        specimenArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        specimenArm.setDirection(DcMotorEx.Direction.FORWARD);

    }


    // Actions are below

    public class speciArmGrab implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            specClawOpen.setPosition(var.specigrabberOpen);
            specClawRotate.setPosition(var.speciRotateGrab);

            return false;

        }
    }
    public Action SpeciArmGrab() {
        return new speciArmGrab();
    }



    public class updatePID implements Action  {
        public updatePID() {
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //SET SLIDE POWERS BASED ON PID CONTROLLER
            specimenArm.setPower(SpecimenArm_PID_Class.returnSpecimenArmPID(setPosition, specimenArm.getCurrentPosition()));
            return true;
        }
    }
    public Action UpdatePID() {
        return new updatePID();
    }



    public class setPosition implements Action  {
        int set;
        public setPosition(int position) {
            set = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setPosition = set;
            return false;
        }
    }
    public Action SetPosition(int pos) {
        return new setPosition(pos);
    }



    public class speciRotateScore implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            specClawRotate.setPosition(var.speciRotateScore);

            return false;

        }
    }
    public Action SpeciRotateScore() {
        return new speciRotateScore();
    }

    public class speciRotateGrab implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            specClawRotate.setPosition(var.speciRotateGrab);

            return false;

        }
    }
    public Action SpeciRotateGrab() {
        return new speciRotateGrab();
    }



    public class specigrabberClose implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            specClawOpen.setPosition(var.specigrabberClosed);

            return false;

        }
    }
    public Action SpecigrabberClose() {
        return new specigrabberClose();
    }



    public class specigrabberOpen implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            specClawOpen.setPosition(var.specigrabberOpen);

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

            specClawOpen.setPosition(distance);

            return false;

        }
    }
    public Action SpecigrabberToDist(int dist) {
        return new specigrabberToDist(dist);
    }

    
    //ADD MORE ACTIONS HERE IF NEEDED
    

    }