package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.PRE_LM1_TESTING.TeleOp;


import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Elbow;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Hand;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM2_SUBSYSTEMS.OuttakeLM2;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Specigrabber;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Wrist;

@Disabled

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

        // Used for robot control
        float x;
        double y;
        double rx;
        double denominator;



        // INSTANTIATE SUBSYSTEMS

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

        OuttakeLM2 outtakeLM2 = new OuttakeLM2(hardwareMap);

        Specigrabber specigrabber = new Specigrabber(hardwareMap);

        // Reverse Motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        lArm.setDirection(DcMotor.Direction.REVERSE);


        // Set Variables
        driveSpeed = 0.5;
        boolean intakeMode = false;
        boolean currentlyIntaking = false;
        boolean gamepadApressed = false;
        int vSlidesPos = 0;
        int hSlidesPos = 0;
        SubsystemsVariables var = new SubsystemsVariables();


        // Reset Encoders
        hSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Set horizontal slides initial pos
        hSlides.setTargetPosition(0);
        hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Sets vertical slides initial pos
        lArm.setTargetPosition(0);
        rArm.setTargetPosition(0);
        lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set lights
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);


        //initialize servos
        Actions.runBlocking(
                new ParallelAction(
                        wrist.WristMiddle(),
                        elbow.ElbowMiddle(),
                        specigrabber.SpecigrabberClose(),
                        outtakeLM2.OuttakeIdle()
                )
        );


        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {

                //Control the claw servo
                if (gamepad1.x || gamepad1.square) {
                    Actions.runBlocking(
                            specigrabber.SpecigrabberClose()
                    );
                }
                if (gamepad1.y || gamepad1.triangle) {
                    Actions.runBlocking(
                            specigrabber.SpecigrabberOpen()
                    );
                }


                // Slides go up when pressing left bumper, but not past the physical max
                if (gamepad1.left_bumper && vSlidesPos < var.vSlidePhysicalMax) {
                    vSlidesPos += 25;
                    lArm.setTargetPosition(vSlidesPos);
                    rArm.setTargetPosition(vSlidesPos);
                    lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                    ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                }

                // Slides go down when pressing right bumper, but not past the physical max of 0
                if (gamepad1.right_bumper && vSlidesPos > 0) {
                    vSlidesPos -= 25;
                    lArm.setTargetPosition(vSlidesPos);
                    rArm.setTargetPosition(vSlidesPos);
                    lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                    ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                }


                // Horizontal Slides go out when pressing the left trigger more than 0.1, but not past the physical max
                if (gamepad1.left_trigger > 0.1 && hSlidesPos < var.hSlideRuleMax) {

                    //Slides move faster the harder you press the trigger
                    hSlidesPos += (int) (8*(gamepad1.left_trigger));

                    hSlides.setTargetPosition(hSlidesPos);
                    hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) hSlides).setVelocity(var.hSlideVelocity);
                }

                // Horizontal Slides go in when pressing the right trigger more than 0.1, but not past the physical max
                if (gamepad1.right_trigger > 0.1 && hSlidesPos > var.hSlideOuttakePos) {

                    //Slides move faster the harder you press the trigger
                    hSlidesPos -= (int) (8*(gamepad1.right_trigger));

                    hSlides.setTargetPosition(hSlidesPos);
                    hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) hSlides).setVelocity(var.hSlideVelocity);
                }


                // Buttons A and B are for intaking and transferring to our outtake
                if ((gamepad1.a || gamepad1.cross) && !gamepadApressed) {  // When A button is newly pressed

                    if(!currentlyIntaking){
                        intakeMode = false;
                    }

                    currentlyIntaking = true;
                    intakeMode = !intakeMode;  // Toggle intake mode

                    Actions.runBlocking(
                            //Stop hand wheels from spinning when the Hslides position is less than 200 and you press A
                            hand.HandStop()
                    );

                    // Run intake actions based on the new intake mode state
                    if (intakeMode && hSlides.getCurrentPosition() > 100) {
                        Actions.runBlocking(
                                new ParallelAction(
                                        wrist.WristIntake(),
                                        elbow.PrepElbowIntake(),
                                        hand.HandIntake()
                                )
                        );
                    } else if (hSlides.getCurrentPosition() > 100){
                        Actions.runBlocking(
                                new ParallelAction(
                                        wrist.WristIntake(),
                                        elbow.ElbowIntake(),
                                        hand.HandIntake()
                                )
                        );
                    }
                }
                // Update the previous state of the A button
                gamepadApressed = gamepad1.a || gamepad1.cross;


                //Transfer game element if the arm is currently intaking and you press B
                if ((gamepad1.b || gamepad1.circle) && currentlyIntaking) {

                    currentlyIntaking = false;

                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            wrist.WristOuttake(),
                                            elbow.ElbowOuttake(),
                                            hand.HandStop(),
                                            hslide.HSlideToTransfer(),
                                            vslides.VSlidesTo0()
                                    ),
                                    hand.HandOuttake()

                            )
                    );

                    //set hSlidesPos so the next time you manually move the hSlides they don't go to your last position
                    //and instead move from the transfer position (which is 'OuttakePos')
                    hSlidesPos = var.hSlideOuttakePos;
                }


                if (gamepad1.dpad_up){
                    Actions.runBlocking(
                            new SequentialAction(
                                    hslide.HSlideToMax()
                            )
                    );

                    //set hSlidesPos so the next time you manually move the hSlides they don't go to your last position
                    //and instead move from the Max position
                    hSlidesPos = var.hSlideRuleMax;
                }


                if (gamepad1.dpad_down){
                    Actions.runBlocking(
                            new SequentialAction(
                                    hslide.HSlideTo0()
                            )
                    );

                    //set hSlidesPos so the next time you manually move the hSlides they don't go to your last position
                    //and instead move from position 0
                    hSlidesPos = 0;
                }


                //Control the outtake servo which rotates the box that stores the game element to deposit it
                if (gamepad1.dpad_left){
                    Actions.runBlocking(
                            new SequentialAction(
                                    outtakeLM2.OuttakeOut()
                            )
                    );
                }
                if (gamepad1.dpad_right){
                    Actions.runBlocking(
                            new SequentialAction(
                                    outtakeLM2.OuttakeIdle()
                            )
                    );
                }


                // Driving Code
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
                telemetry.addData("HSlidesPosVariable = ", hSlidesPos);
                telemetry.update();
            }
        }
    }
}