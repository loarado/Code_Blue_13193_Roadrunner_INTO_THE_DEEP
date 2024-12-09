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
import com.acmerobotics.dashboard.config.Config;

@Disabled
@Config
@Autonomous(name = "PIDF 2 Arms")
public class PIDF_2Arm extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.0065, i = 0, d = 0.000001;
    public static double f = 0.00007;
    public static double relativeP = 0.003;

    public static int target = 1000;

    public final double ticks_in_degree = 145.1/360;

    private DcMotorEx rarm;
    private DcMotorEx larm;

    @Override
    public void runOpMode() {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rarm = hardwareMap.get(DcMotorEx.class, "rArm");
        larm = hardwareMap.get(DcMotorEx.class, "lArm");

        double leftPower;
        double rightPower;
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        larm.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {




            while (opModeIsActive()) {
                controller.setPIDF(p, i, d, f);
                double motorRelativeError = Math.abs(larm.getCurrentPosition()-rarm.getCurrentPosition())>1?larm.getCurrentPosition()-rarm.getCurrentPosition():0;
                double power = controller.calculate(getCurrentPosition(), target);

                leftPower = power-relativeP*motorRelativeError;
                rightPower = power+relativeP*motorRelativeError;
                double denom = Math.max(leftPower, Math.max(rightPower, 1));

                larm.setPower(leftPower / denom);
                rarm.setPower (rightPower / denom);

                int rArmPos = rarm.getCurrentPosition();
                int lArmPos = larm.getCurrentPosition();

                telemetry.addData("r pos ", rArmPos);
                telemetry.addData("l pos ", lArmPos);
                telemetry.addData("target ", target);
                telemetry.addData("power ", power);

                telemetry.update();
            }
        }
    }

    private double getCurrentPosition() {
        return (double) (larm.getCurrentPosition() + rarm.getCurrentPosition()) /2;
    }
}

