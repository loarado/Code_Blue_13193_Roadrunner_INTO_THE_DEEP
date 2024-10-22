package org.firstinspires.ftc.teamcode.teleops;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;



@TeleOp(name = "TeleOp Test")
public class TeleOpTest extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor rArm;
    private DcMotor hSlides;

    @Override
    public void runOpMode() {
        double speed;
        float x;
        double y;
        double rx;
        double denominator;


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rArm = hardwareMap.get(DcMotor.class, "rArm");
        hSlides = hardwareMap.get(DcMotor.class, "hSlides");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            speed = 0.5;
            while (opModeIsActive()) {

                x = -(gamepad1.left_stick_x * 1);
                y = gamepad1.left_stick_y * -1.1;
                rx = gamepad1.right_stick_x * 0.75;
                denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(x), Math.abs(y), Math.abs(rx))), speed));
                // Powers motors
                leftBack.setPower((y + x + rx) / denominator);
                leftFront.setPower(((y - x) + rx) / denominator);
                rightBack.setPower(((y - x) - rx) / denominator);
                rightFront.setPower(((y + x) - rx) / denominator);
                if(gamepad1.dpad_down){
                    hSlides.setPower(0.4);
                } else if(gamepad1.dpad_up){
                    hSlides.setPower(-0.4);
                } else if (gamepad1.dpad_left) {
                    rArm.setPower(0.4);
                } else if (gamepad1.dpad_right) {
                    rArm.setPower(-0.4);
                }
                // Displays x, y, and rx
                telemetry.addData("x = ", x);
                telemetry.addData("y = ", y);
                telemetry.addData("rx = ", rx);
                telemetry.update();
            }
        }
    }
}
