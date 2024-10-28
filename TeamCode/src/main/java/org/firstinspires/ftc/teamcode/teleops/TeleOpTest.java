package org.firstinspires.ftc.teamcode.teleops;


import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Elbow;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Hand;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Outtake;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Specigrabber;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.SubsystemsVariables;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Wrist;


@TeleOp(name = "TeleOp AndroidStudio Test", group = "TeleOp")
public class TeleOpTest extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor lArm;
    private DcMotor hSlides;
    private DcMotor rArm;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private RevBlinkinLedDriver lights;

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
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        HorizontalSlides hslide = new HorizontalSlides(hardwareMap);

        VerticalSlides vslides = new VerticalSlides(hardwareMap);

        Elbow elbow = new Elbow(hardwareMap);

        Wrist wrist = new Wrist(hardwareMap);

        Hand hand = new Hand(hardwareMap);

        Outtake outtake = new Outtake(hardwareMap);

        Specigrabber specigrabber = new Specigrabber(hardwareMap);

        // Reverse Motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        lArm.setDirection(DcMotor.Direction.REVERSE);
        // Set Variables
        driveSpeed = 0.5;
        boolean intakeMode = false;
        boolean currentlyIntaking = false;

        SubsystemsVariables var = new SubsystemsVariables();

        // Reset Encoders
        hSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Sets slides initial pos
        hSlides.setTargetPosition(0);
        hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Sets arms initial pos
        lArm.setTargetPosition(0);
        rArm.setTargetPosition(0);
        lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Sets hSlides to inside
                if (gamepad1.x) {
                    Actions.runBlocking(
                    specigrabber.SpecigrabberClose()
                    );
                }
                // Extends hSlides out
                if (gamepad1.y) {
                    Actions.runBlocking(
                            specigrabber.SpecigrabberOpen()
                    );
                }
                // Sets vertical slides to down position
                if (gamepad1.left_bumper &&  lArm.getCurrentPosition() < var.vSlidePhysicalMax && rArm.getCurrentPosition() < var.vSlidePhysicalMax) {
                        lArm.setTargetPosition(lArm.getCurrentPosition()+10);
                        rArm.setTargetPosition(rArm.getCurrentPosition()+10);
                        lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                        ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                }
                // Sets vertical slides to up position
                if (gamepad1.right_bumper && lArm.getCurrentPosition() > 0 && rArm.getCurrentPosition() > 0 ) {
                        lArm.setTargetPosition(lArm.getCurrentPosition()-10);
                        rArm.setTargetPosition(rArm.getCurrentPosition()-10);
                        lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                        ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                }
                // Inputs one block
                if (gamepad1.a) {
                    currentlyIntaking = true;
                    if(hSlides.getCurrentPosition()>200 && !intakeMode) {
                        intakeMode = true;
                        Actions.runBlocking(
                                new ParallelAction(
                                        wrist.WristIntake(),
                                        elbow.PrepElbowIntake(),
                                        hand.HandIntake()
                                )
                        );

                    }
                    else {
                        intakeMode = false;
                        Actions.runBlocking(
                                new ParallelAction(
                                        wrist.WristIntake(),
                                        elbow.ElbowIntake(),
                                        hand.HandIntake()
                                )
                        );
                    }
                }
                if (gamepad1.b && currentlyIntaking) {
                    currentlyIntaking = false;
                        Actions.runBlocking(
                                new SequentialAction(
                                new ParallelAction(
                                        wrist.WristOuttake(),
                                        elbow.ElbowOuttake(),
                                        hand.HandStop(),
                                        hslide.HSlideTo0(),
                                        vslides.VSlidesTo0()
                                ),
                                hand.HandOuttake()
                                )
                        );
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
                // Displays x, y, rx, rightVerticalSlidePos, leftVerticalSlidePos, and horizontalSlidesPos
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