package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.teamcode.tuning.pidTest.V_Slides_PID_Class;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;

public class VerticalSlides {
    private PIDController controller;
    
    public static double p = 0.0065, i = 0, d = 0.000001;
    public static double f = 0.00007;
    public static double relativeP = 0.003;

    public static int target = 0;

    public final double ticks_in_degree = 145.1/360;

    
    
    double leftPower;
    double rightPower;

    //Instantiate the sensor/servo/motor
    public DcMotorEx lArm;
    public DcMotorEx rArm;
    public TouchSensor vTouchLeft;
    public TouchSensor vTouchRight;

    // Import final variables
    SubsystemsVariables var = new SubsystemsVariables();

    HardwareMap hardwareMapR;

    public VerticalSlides(HardwareMap hardwareMap) {

        hardwareMapR = hardwareMap;

        //Constructor
        lArm = hardwareMap.get(DcMotorEx.class, "lArm");
        lArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lArm.setDirection(DcMotor.Direction.REVERSE);
        rArm = hardwareMap.get(DcMotorEx.class, "rArm");
        rArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rArm.setDirection(DcMotor.Direction.FORWARD);

        vTouchLeft = hardwareMap.get(TouchSensor.class, "vTouchLeft");
        vTouchRight = hardwareMap.get(TouchSensor.class, "vTouchRight");

    }



    public int getCurrentPosition(){
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
                rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }else if(vTouchRight.isPressed()){
                rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //WAIT FOR SLIDES TO REACH POSITION

            return (lArm.isBusy()||rArm.isBusy());
        }
    }
    public Action VSlideToMax() {
        return new vSlidesToMax();
    }


    public void stopSlides() {
        rArm.setPower(0);
        lArm.setPower(0);
    }



    public class vSlidesToDistPIDF implements Action  {

        int distance = 0;
        double velocity = var.vSlideVelocity;
        int distanceTolerance = 20;

        public vSlidesToDistPIDF(int dist) {

            //Constructor to set "distance", can't be above physical max

            if(dist<=var.vSlidePhysicalMax) {
                distance = dist;
            }else{
                distance = var.vSlidePhysicalMax;
            }

            controller = new PIDController(p, i, d);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {


            //SET SLIDE POWERS BASED ON PID CONTROLLER
            rArm.setPower(V_Slides_PID_Class.returnRightVSlidePID(distance, lArm.getCurrentPosition(), rArm.getCurrentPosition()));
            lArm.setPower(V_Slides_PID_Class.returnLeftVSlidePID(distance, lArm.getCurrentPosition(), rArm.getCurrentPosition()));


                distanceTolerance = 20;


            //WHILE SLIDES ARE NOT AT TARGET
            if((lArm.getCurrentPosition()>(distance-distanceTolerance)&&
                lArm.getCurrentPosition()<(distance+distanceTolerance))&&
                (rArm.getCurrentPosition()>(distance-distanceTolerance)&&
                rArm.getCurrentPosition()<(distance+distanceTolerance))
                    //IF BOTH SLIDES ARE WITHIN TARGET DISTANCE
            ){
                rArm.setPower(V_Slides_PID_Class.returnRightVSlidePID(distance, distance, distance));
                lArm.setPower(V_Slides_PID_Class.returnLeftVSlidePID(distance, distance, distance));
                return false;
            }else{
                return true;
            }

        }
    }
    public Action VSlidesToDistPIDF(int dist) {
        return new vSlidesToDistPIDF(dist);
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

            rArm.setTargetPosition(distance);
            lArm.setTargetPosition(distance);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) lArm).setVelocity(velocity);
            ((DcMotorEx) rArm).setVelocity(velocity);

            if(distance<50){
                if(vTouchLeft.isPressed()){
                    lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }else if(vTouchRight.isPressed()){
                    rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }

            //WAIT FOR SLIDES TO REACH POSITION

            return (lArm.isBusy()&&rArm.isBusy());
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
                rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }else if(vTouchRight.isPressed()){
                rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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