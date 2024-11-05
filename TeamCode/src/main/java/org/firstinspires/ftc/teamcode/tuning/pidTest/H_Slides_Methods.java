package org.firstinspires.ftc.teamcode.tuning.pidTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled

@Config
public class H_Slides_Methods {
    private PIDController controller;
    private static PIDController controllerSend;

    public static double p = 0.006, i = 0, d = 0.0002;

    public static double f = 0;

    public static final double ticks_in_degree = 145.1/360;

    public static double returnPID(double currentPos, double target){
        controllerSend = new PIDController(p, i, d);
        controllerSend.setPID(p, i, d);
        double pid = controllerSend.calculate(currentPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        return pid + ff;
    }




}

