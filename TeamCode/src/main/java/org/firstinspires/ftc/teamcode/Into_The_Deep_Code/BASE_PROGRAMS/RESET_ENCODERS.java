package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.BASE_PROGRAMS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "RESET ENCODERS")
public class RESET_ENCODERS extends LinearOpMode {


    private DcMotorEx hSlides;
    private DcMotorEx rArm;
    private DcMotorEx lArm;
    private DcMotorEx specimenArm;
    @Override
    public void runOpMode() {



        hSlides = hardwareMap.get(DcMotorEx.class, "hSlides");
        hSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rArm = hardwareMap.get(DcMotorEx.class, "rArm");
        rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lArm = hardwareMap.get(DcMotorEx.class, "lArm");
        lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimenArm = hardwareMap.get(DcMotorEx.class, "specimenArm");
        specimenArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {

                telemetry.addData("uh we did it yay", 1);

                telemetry.update();
            }
        }
    }
}

