package org.firstinspires.ftc.teamcode.teleops;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;



@TeleOp(name = "TeleOp AndroidStudio Test", group = "TeleOp")
public class TeleOpTest extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor lArm;
    private DcMotor hSlides;
    private DcMotor rArm;
    private DcMotor rightBack;
    private DcMotor rightFront;

    @Override
    public void runOpMode() {
        double driveSpeed;
        int hSlidesVelocity;
        int armsVelocity;
        int hSlidesInitialPos;
        int hSlidesOutPos;
        int armsDownPos;
        int armsUpPos;
        float x;
        double y;
        double rx;
        double denominator;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        lArm = hardwareMap.get(DcMotor.class, "lArm");
        hSlides = hardwareMap.get(DcMotor.class, "hSlides");
        rArm = hardwareMap.get(DcMotor.class, "rArm");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        // Reverse Motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        lArm.setDirection(DcMotor.Direction.REVERSE);
        // Set Variables
        driveSpeed = 0.5;
        hSlidesVelocity = 1300;
        armsVelocity = 1300;
        hSlidesInitialPos = 0;
        hSlidesOutPos = 700;
        armsDownPos = 0;
        armsUpPos = 2000;
        // Reset Encoders
        hSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Sets slides initial pos
        hSlides.setTargetPosition(hSlidesInitialPos);
        hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Sets arms initial pos
        lArm.setTargetPosition(armsDownPos);
        rArm.setTargetPosition(armsDownPos);
        lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Sets hSlides to inside
                if (gamepad1.a) {
                    hSlides.setTargetPosition(hSlidesInitialPos);
                    hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) hSlides).setVelocity(hSlidesVelocity);
                }
                // Extends hSlides out
                if (gamepad1.b) {
                    hSlides.setTargetPosition(hSlidesOutPos);
                    hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) hSlides).setVelocity(hSlidesVelocity);
                }
                // Sets arms to down position
                if (gamepad1.left_bumper) {
                    lArm.setTargetPosition(armsDownPos);
                    rArm.setTargetPosition(armsDownPos);
                    lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) lArm).setVelocity(armsVelocity);
                    ((DcMotorEx) rArm).setVelocity(armsVelocity);
                }
                // Sets arms to up position
                if (gamepad1.right_bumper) {
                    lArm.setTargetPosition(armsUpPos);
                    rArm.setTargetPosition(armsUpPos);
                    lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) lArm).setVelocity(armsVelocity);
                    ((DcMotorEx) rArm).setVelocity(armsVelocity);
                }
                // Driving
                x = -(gamepad1.left_stick_x * 1);
                y = -(gamepad1.left_stick_y * 1.1);
                rx = gamepad1.right_stick_x * 0.75;
                denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(x), Math.abs(y), Math.abs(rx))), driveSpeed));
                // Powers wheel motors
                leftBack.setPower((y + x + rx) / denominator);
                leftFront.setPower(((y - x) + rx) / denominator);
                rightBack.setPower(((y - x) - rx) / denominator);
                rightFront.setPower(((y + x) - rx) / denominator);
                // Displays x, y, and rx
                telemetry.addData("x = ", x);
                telemetry.addData("y = ", y);
                telemetry.addData("rx = ", rx);
                telemetry.addData("currentRightArmPos = ", rArm.getCurrentPosition());
                telemetry.addData("currentLeftArmPos = ", lArm.getCurrentPosition());
                telemetry.addData("currentHorizontalSlidesPos = ", hSlides.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}