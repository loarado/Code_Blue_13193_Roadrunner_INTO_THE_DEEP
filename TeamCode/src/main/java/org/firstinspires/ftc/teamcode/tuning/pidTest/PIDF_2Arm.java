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
@Autonomous(name = "PIDF 2 Arms")
public class PIDF_2Arm extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.01, i = 0, d = 0.0002;
    public static double f = 0.3;

    public static int target = 500;

    public final double ticks_in_degree = 145.1/360;

    private DcMotorEx rarm;
    private DcMotorEx larm;

    @Override
    public void runOpMode() {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rarm = hardwareMap.get(DcMotorEx.class, "rArm");
        larm = hardwareMap.get(DcMotorEx.class, "rArm");
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        larm.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {



            while (opModeIsActive()) {
                controller.setPID(p, i, d);
                int armPos = rarm.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

                double power = pid + ff;

                rarm.setPower(power);
                larm.setPower(-power);

                telemetry.addData("pos ", armPos);
                telemetry.addData("target ", target);
                telemetry.addData("power ", power);

                telemetry.update();
            }
        }
    }
}

