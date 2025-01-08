package org.firstinspires.ftc.teamcode.tuning.pidTest;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;


@Config
public class SpecimenArm_PID_Class {

    private static PIDController controller;


    public static double p = 0.005, i = 0.075, d = 0.0002;
    public static double f = 0.05;
    public static final double ticks_in_degree = (2786.2/360)/2;

    static double power;

    public static double returnSpecimenArmPID(double target, double specArmPos){

        controller = new PIDController(p, i, d);

        controller.setPID(p, i, d);

        double pid = controller.calculate(specArmPos, target);

        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        power = pid + ff;

        return power;
       
    }

}

