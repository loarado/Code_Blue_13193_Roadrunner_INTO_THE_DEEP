package org.firstinspires.ftc.teamcode.tuning.pidTest;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;


@Config
public class V_Slides_PID_Class {

    private static PIDController controller;


    public static double p = 0.0065, i = 0, d = 0.000001;
    public static double f = 0.00007;
    public static double relativeP = 0.003;

    static double leftPower;
    static double rightPower;

    public static double returnLeftVSlidePID(double target, double lArmPos, double rArmPos){

        controller = new PIDController(p, i, d);

        controller.setPIDF(p, i, d, f);
        double motorRelativeError = Math.abs(lArmPos-rArmPos)>1?lArmPos-rArmPos:0;
        double power = controller.calculate((lArmPos+rArmPos)/2, target);
        leftPower = power-relativeP*motorRelativeError;
        rightPower = power+relativeP*motorRelativeError;
        double denom = Math.max(leftPower, Math.max(rightPower, 1));

        return leftPower/denom;
       
    }

    public static double returnRightVSlidePID(double target, double lArmPos, double rArmPos){

        controller = new PIDController(p, i, d);

        controller.setPIDF(p, i, d, f);
        double motorRelativeError = Math.abs(lArmPos-rArmPos)>1?lArmPos-rArmPos:0;
        double power = controller.calculate((lArmPos+rArmPos)/2, target);
        leftPower = power-relativeP*motorRelativeError;
        rightPower = power+relativeP*motorRelativeError;
        double denom = Math.max(leftPower, Math.max(rightPower, 1));

        return rightPower/denom;

    }

}

