package org.firstinspires.ftc.teamcode.tuning.pidTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "PIDF RESET ENCODERS")
public class PIDF_RESET_ENCODERS extends LinearOpMode {


    private DcMotorEx hSlides;
    private DcMotorEx rArm;
    private DcMotorEx lArm;
    @Override
    public void runOpMode() {



        hSlides = hardwareMap.get(DcMotorEx.class, "hSlides");
        hSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rArm = hardwareMap.get(DcMotorEx.class, "rArm");
        rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lArm = hardwareMap.get(DcMotorEx.class, "lArm");
        lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {

                telemetry.addData("uh we did it wow", 1);

                telemetry.update();
            }
        }
    }
}

