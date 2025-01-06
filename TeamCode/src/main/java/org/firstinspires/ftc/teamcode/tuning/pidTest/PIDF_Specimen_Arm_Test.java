package org.firstinspires.ftc.teamcode.tuning.pidTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;



@Config
@Autonomous(name = "PIDF Specimen Arm Test")
public class PIDF_Specimen_Arm_Test extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.01, i = 0, d = 0;
    public static double f = 1.746;
    // make f mass*g*L then tune it




    public static final double ticks_in_degree = 2786.2/360;


    public static int target = 90 * 8;

    private DcMotorEx arm;

    @Override
    public void runOpMode() {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hardwareMap.get(DcMotorEx.class, "specimenArm");
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {



            while (opModeIsActive()) {
                controller.setPID(p, i, d);
                int armPos = arm.getCurrentPosition();
                double pid = controller.calculate(armPos, target);

                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
                //formula for ff is mass (kg) * g (9.81) * L (distance in m) * cos(angle)
                // left side of equation is f, can be tuned but for this specimen arm we can use a base value calculated by the formula
                // angle can be found by dividing the target by ticks in degree.
                // I understand it now

                double power = pid + ff;

                arm.setPower(power);

                telemetry.addData("pos ", armPos);
                telemetry.addData("target ", target);
                telemetry.addData("power ", power);

                telemetry.update();
            }
        }
    }
}

