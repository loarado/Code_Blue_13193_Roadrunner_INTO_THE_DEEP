package org.firstinspires.ftc.teamcode.tuning.pidTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled

@Config
@Autonomous(name = "PIDF H Slides")
public class PIDF_H_Slides extends LinearOpMode {
    private PIDController controller;
    private static PIDController controllerSend;

    public static double p = 0.006, i = 0, d = 0.0002;
    public static double f = 0;

    public static int target = 300;

    public static final double ticks_in_degree = 145.1/360;

    private DcMotorEx hSlides;

    public static double returnPID(double currentPos, double target){
        controllerSend = new PIDController(p, i, d);
        controllerSend.setPID(p, i, d);
        double pid = controllerSend.calculate(currentPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        return pid + ff;
    }

    @Override
    public void runOpMode() {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hSlides = hardwareMap.get(DcMotorEx.class, "hSlides");
        //hSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        if (opModeIsActive()) {



            while (opModeIsActive()) {
                controller.setPID(p, i, d);
                int armPos = hSlides.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

                double power = pid + ff;

                hSlides.setPower(power);

                telemetry.addData("pos ", armPos);
                telemetry.addData("target ", target);
                telemetry.addData("power ", power);

                telemetry.update();
            }
        }
    }
}

