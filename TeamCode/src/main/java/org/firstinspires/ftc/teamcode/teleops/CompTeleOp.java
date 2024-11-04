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


@TeleOp(name = "Into The Deep TeleOp", group = "TeleOp")
public class CompTeleOp extends LinearOpMode {

    // Motor declarations
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

        double driveSpeed; // Variable for controlling robot driving speed

        // Variables for robot movement
        float x;
        double y;
        double rx;
        double denominator;

        // Instantiate subsystem classes, which control specific robot mechanisms:
        // HorizontalSlides: Controls horizontal movement of slides
        // VerticalSlides: Controls vertical movement of slides
        // Elbow: Controls elbow component of the arm
        // Wrist: Controls wrist position
        // Hand: Controls hand mechanism for intaking game elements
        // Outtake: Controls outtake servo for depositing elements
        // Specigrabber: Controls claw mechanism for grabbing elements
        HorizontalSlides hslide = new HorizontalSlides(hardwareMap);
        VerticalSlides vslides = new VerticalSlides(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Hand hand = new Hand(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Specigrabber specigrabber = new Specigrabber(hardwareMap);

        // 'var' holds subsystem-specific variables such as physical limits and velocities
        // for easier access and adjustment across the tele-op program
        SubsystemsVariables var = new SubsystemsVariables();

        // Hardware initialization
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        lArm = hardwareMap.get(DcMotor.class, "lArm");
        hSlides = hardwareMap.get(DcMotor.class, "hSlides");
        rArm = hardwareMap.get(DcMotor.class, "rArm");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        lArm.setDirection(DcMotor.Direction.REVERSE);

        // Initialize control variables
        driveSpeed = 0.45; // Initial drive speed factor
        boolean intakeMode = false; // Tracks intake mode state (on/off)
        boolean currentlyIntaking = false; // Indicates if intake is active
        boolean gamepadApressed = false; // Tracks A button state
        int vSlidesPos = 0; // Variable for vertical slides position
        int hSlidesPos = 0; // Variable for horizontal slides position

        // Reset encoders to set initial motor positions
        hSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set horizontal slides initial position
        hSlides.setTargetPosition(0);
        hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set vertical slides initial position
        lArm.setTargetPosition(0);
        rArm.setTargetPosition(0);
        lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set initial LED pattern
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

        // Initialize servos to default positions using ParallelAction
        Actions.runBlocking(
                new ParallelAction(
                        wrist.WristMiddle(),
                        elbow.ElbowMiddle(),
                        specigrabber.SpecigrabberClose(),
                        outtake.OuttakeIdle()
                )
        );

        // Wait for start button to be pressed
        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {

                // Claw control - closes claw if X is pressed, opens if Y is pressed
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

                // Vertical slide control - moves slides up when left bumper is pressed,
                // but not past the maximum allowed height
                if (gamepad1.left_bumper && vSlidesPos < var.vSlidePhysicalMax) {
                    vSlidesPos += 25;
                    lArm.setTargetPosition(vSlidesPos);
                    rArm.setTargetPosition(vSlidesPos);
                    lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                    ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                }

                // Moves slides down when right bumper is pressed, but stops at minimum height
                if (gamepad1.right_bumper && vSlidesPos > 0) {
                    vSlidesPos -= 25;
                    lArm.setTargetPosition(vSlidesPos);
                    rArm.setTargetPosition(vSlidesPos);
                    lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                    ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                }

                // Horizontal slides control - moves slides outward with left trigger,
                // increases speed with harder press, but stops at max position
                if (gamepad1.left_trigger > 0.1 && hSlidesPos < var.hSlidePhysicalMax) {
                    hSlidesPos += (int) (8 * (gamepad1.left_trigger));
                    hSlides.setTargetPosition(hSlidesPos);
                    hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) hSlides).setVelocity(var.hSlideVelocity);
                }

                // Moves slides inward with right trigger, but stops at minimum position
                if (gamepad1.right_trigger > 0.1 && hSlidesPos > var.hSlideOuttakePos) {
                    hSlidesPos -= (int) (8 * (gamepad1.right_trigger));
                    hSlides.setTargetPosition(hSlidesPos);
                    hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) hSlides).setVelocity(var.hSlideVelocity);
                }

                // Intake mode toggle with A button
                if ((gamepad1.a || gamepad1.cross) && !gamepadApressed) {  // When A button is newly pressed
                    if (!currentlyIntaking) {  // Ensures prep mode is first if idle
                        intakeMode = false;
                    }
                    currentlyIntaking = true;
                    intakeMode = !intakeMode;  // Toggle intake mode

                    Actions.runBlocking(
                            hand.HandStop()  // Stops hand wheel spinning if hSlides position is low and A is pressed
                    );

                    // Adjusts wrist and elbow positions based on intakeMode state
                    if (intakeMode && hSlides.getCurrentPosition() > 100) {
                        Actions.runBlocking(
                                new ParallelAction(
                                        wrist.WristIntake(),
                                        elbow.PrepElbowIntake(),
                                        hand.HandIntake()
                                )
                        );
                    } else if (hSlides.getCurrentPosition() > 100) {
                        Actions.runBlocking(
                                new ParallelAction(
                                        wrist.WristIntake(),
                                        elbow.ElbowIntake(),
                                        hand.HandIntake()
                                )
                        );
                    }
                }
                gamepadApressed = gamepad1.a || gamepad1.cross;

                // Transfer action with B button if intake is active
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

                    // Resets hSlides position variable to OuttakePos for consistency
                    hSlidesPos = var.hSlideOuttakePos;
                }

                // Adjust horizontal slides with dpad up (to max) and down (to 0)
                if (gamepad1.dpad_up) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    hslide.HSlideToMax()
                            )
                    );
                    hSlidesPos = var.hSlideRuleMax;
                }
                if (gamepad1.dpad_down) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    hslide.HSlideToTransfer()
                            )
                    );
                    hSlidesPos = var.hSlideOuttakePos;
                }

                // Controls outtake position with dpad left (deposit) and right (idle)
                if (gamepad1.dpad_left&&lArm.getCurrentPosition()>500&&specigrabber.specigrabber.getPosition()>0.7) {
                    //If the robot wont break, move the outtake box

                    Actions.runBlocking(
                            new SequentialAction(
                                    outtake.OuttakeOut()
                            )
                    );
                }
                if (gamepad1.dpad_right) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    outtake.OuttakeIdle()
                            )
                    );
                }

                if(hSlides.getCurrentPosition()>300||lArm.getCurrentPosition()>1400||rArm.getCurrentPosition()>1400||currentlyIntaking){
                    driveSpeed = 0.3;
                } else{
                    driveSpeed = 0.45;
                }

                // Drive controls
                x = -gamepad1.left_stick_x;
                y = gamepad1.left_stick_y;
                rx = -gamepad1.right_stick_x;

                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                leftFront.setPower((y + x + rx) / denominator * driveSpeed);
                leftBack.setPower((y - x + rx) / denominator * driveSpeed);
                rightFront.setPower((y - x - rx) / denominator * driveSpeed);
                rightBack.setPower((y + x - rx) / denominator * driveSpeed);
            }
        }
    }
}