package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;

public class VerticalSlides {

    //Instantiate the sensor/servo/motor
    public DcMotor lArm;
    public DcMotor rArm;
    public TouchSensor vTouchLeft;
    public TouchSensor vTouchRight;

    // Import final variables
    SubsystemsVariables var = new SubsystemsVariables();


    public VerticalSlides(HardwareMap hardwareMap) {

        //Constructor
        lArm = hardwareMap.get(DcMotor.class, "lArm");
        lArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lArm.setDirection(DcMotor.Direction.REVERSE);
        rArm = hardwareMap.get(DcMotor.class, "rArm");
        rArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rArm.setDirection(DcMotor.Direction.FORWARD);

        vTouchLeft = hardwareMap.get(TouchSensor.class, "vTouchLeft");
        vTouchRight = hardwareMap.get(TouchSensor.class, "vTouchRight");

    }

    public int getCurrentPosition(){
        if(vTouchLeft.isPressed()){
            lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }else if(vTouchRight.isPressed()){
            rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        return (lArm.getCurrentPosition() + rArm.getCurrentPosition())/2;
    }

    // Actions are below
    
    public class vSlidesToMax implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            rArm.setTargetPosition(var.vSlidePhysicalMax);
            lArm.setTargetPosition(var.vSlidePhysicalMax);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
            ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);

            if(vTouchLeft.isPressed()){
                lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }else if(vTouchRight.isPressed()){
                rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //WAIT FOR SLIDES TO REACH POSITION

            return (lArm.isBusy()||rArm.isBusy());
        }
    }
    public Action VSlideToMax() {
        return new vSlidesToMax();
    }



    public class vSlidesToDist implements Action  {

        int distance = 0;
        double velocity = var.vSlideVelocity;

        public vSlidesToDist(int dist) {

            //Constructor to set "distance", can't be above physical max

            if(dist<=var.vSlidePhysicalMax) {
                distance = dist;
            }else{
                distance = var.vSlidePhysicalMax;
            }

        }
        public vSlidesToDist(int dist, double velo) {

            //Constructor to set "distance", can't be above physical max

            if(dist<=var.vSlidePhysicalMax) {
                distance = dist;
            }else{
                distance = var.vSlidePhysicalMax;
            }

            if(velo>0 && velo<var.vSlideVelocity) {
                velocity = velo;
            }else{
                velocity = var.vSlideVelocity;
            }

        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if(vTouchLeft.isPressed()){
                lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }else if(vTouchRight.isPressed()){
                rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            rArm.setTargetPosition(distance);
            lArm.setTargetPosition(distance);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) lArm).setVelocity(velocity);
            ((DcMotorEx) rArm).setVelocity(velocity);

            //WAIT FOR SLIDES TO REACH POSITION

            return (lArm.isBusy()||rArm.isBusy());
        }
    }
    public Action VSlidesToDist(int dist, double velo) {
        return new vSlidesToDist(dist, velo);
    }
    public Action VSlidesToDist(int dist) {
        return new vSlidesToDist(dist);
    }



    public class vSlidesTo0 implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if(vTouchLeft.isPressed()){
                lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }else if(vTouchRight.isPressed()){
                rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            rArm.setTargetPosition(0);
            lArm.setTargetPosition(0);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
            ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);

            //WAIT FOR SLIDES TO REACH POSITION

            return (lArm.isBusy()||rArm.isBusy());
        }
    }
    public Action VSlidesTo0() {
        return new vSlidesTo0();
    }


    //ADD MORE ACTIONS HERE IF NEEDED


    }