package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizontalSlides {

    //Instantiate the sensor/servo/motor
    public DcMotor hSlides;

    // Import final variables
    SubsystemsVariables var = new SubsystemsVariables();


    public HorizontalSlides(HardwareMap hardwareMap) {

        //Constructor
        hSlides = hardwareMap.get(DcMotor.class, "hSlides");
        hSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hSlides.setDirection(DcMotor.Direction.FORWARD);

    }

    // Actions are below

    public class hSlideToMax implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            hSlides.setTargetPosition(var.hSlideRuleMax);
            hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hSlides.setPower(var.hSlideSpeed);

            //WAIT FOR SLIDE TO REACH POSITION

            return hSlides.isBusy();
        }
    }
    public Action HSlideToMax() {
        return new hSlideToMax();
    }



    public class hSlideTo0 implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            hSlides.setTargetPosition(0);
            hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hSlides.setPower(var.hSlideSpeed);

            //WAIT FOR SLIDE TO REACH POSITION

            return hSlides.isBusy();

        }
    }
    public Action HSlideTo0() {
        return new hSlideTo0();
    }



    public class hSlideToDist implements Action  {

        int distance = 0;

        public hSlideToDist(int dist) {

            //Constructor to set "distance", can't be above physical max

            if(dist<=var.hSlidePhysicalMax) {
                distance = dist;
            }else{
                distance = var.hSlidePhysicalMax;
            }
        }

        @Override

        public boolean run(@NonNull TelemetryPacket packet) {

            hSlides.setTargetPosition(distance);
            hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hSlides.setPower(var.hSlideSpeed);

            //WAIT FOR SLIDE TO REACH POSITION

            return hSlides.isBusy();
        }
    }
    public Action HSlideToDist(int dist) {
        return new hSlideToDist(dist);
    }


    //ADD MORE ACTIONS HERE IF NEEDED


    }