package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VerticalSlides {

    //Instantiate the sensor/servo/motor
    public DcMotor lArm;
    public DcMotor rArm;

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

    }

    // Actions are below
    
    public class vSlidesToMax implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            rArm.setTargetPosition(var.vSlidePhysicalMax);
            lArm.setTargetPosition(var.vSlidePhysicalMax);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rArm.setPower(var.vSlideSpeed);
            lArm.setPower(var.vSlideSpeed);

            //WAIT FOR SLIDES TO REACH POSITION

            return (lArm.isBusy()&&rArm.isBusy());
        }
    }
    public Action VSlideToMax() {
        return new vSlidesToMax();
    }



    public class vSlidesToDist implements Action  {

        int distance = 0;

        public vSlidesToDist(int dist) {

            //Constructor to set "distance", can't be above physical max

            if(dist<=var.vSlidePhysicalMax) {
                distance = dist;
            }else{
                distance = var.vSlidePhysicalMax;
            }

        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            rArm.setTargetPosition(distance);
            lArm.setTargetPosition(distance);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rArm.setPower(var.vSlideSpeed);
            lArm.setPower(var.vSlideSpeed);

            //WAIT FOR SLIDES TO REACH POSITION

            return (lArm.isBusy()&&rArm.isBusy());
        }
    }
    public Action VSlidesToDist(int dist) {
        return new vSlidesToDist(dist);
    }



    public class vSlidesTo0 implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            rArm.setTargetPosition(0);
            lArm.setTargetPosition(0);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rArm.setPower(var.vSlideSpeed);
            lArm.setPower(var.vSlideSpeed);

            //WAIT FOR SLIDES TO REACH POSITION

            return (lArm.isBusy()&&rArm.isBusy());
        }
    }
    public Action VSlidesTo0() {
        return new vSlidesTo0();
    }


    //ADD MORE ACTIONS HERE IF NEEDED


    }