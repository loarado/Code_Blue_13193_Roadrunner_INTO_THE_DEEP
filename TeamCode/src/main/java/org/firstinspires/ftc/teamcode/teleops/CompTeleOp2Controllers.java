package org.firstinspires.ftc.teamcode.teleops;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Elbow;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Hand;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Outtake;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Specigrabber;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.SubsystemsVariables;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.Wrist;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "2 Drivers Comp TeleOp", group = "TeleOp")
public class CompTeleOp2Controllers extends LinearOpMode {

    /*

                         READ ME

    BEFORE EDITING THIS TELE-OP, MAKE SURE TO READ THE
             " INTO THE DEEP TELE-OP CONTROLS "
    DRIVE FILE IN THE GOOGLE DRIVE, THIS LISTS WHAT WE SHOULD
    HAVE EACH BUTTON DO TO OPTIMIZE THE TELE-OP PERIOD.

     */

    // Motor declarations
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor lArm;
    private DcMotor hSlides;
    private DcMotor rArm;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private RevBlinkinLedDriver lights;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() {


        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


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
        driveSpeed = 0.5; // Initial drive speed factor
        boolean intakeMode = false; // Tracks intake mode state (on/off)
        boolean currentlyIntaking = false; // Indicates if intake is active
        boolean SampleTransferred = false; // Indicates if a transfer has occurred and if we have a sample in our outtake
        boolean gamepadApressed = false; // Tracks A button state
        boolean gamepadXpressed = false; // Tracks Y button state
        boolean dPadDownPressed = false; // Tracks DpadDpwn button state
        boolean dPadRightPressed = false; // Tracks DpadRight button state
        boolean optionsPressed = false;
        int vSlidesPos = 0; // Variable for vertical slides position
        int hSlidesPos = 0; // Variable for horizontal slides position

        boolean SpecimenMode = false;
        boolean BasketMode = true;
        
        boolean outtakeIsOut = false;
        boolean specigrabberIsOpen = false;

        VelConstraint velSlow = new TranslationalVelConstraint(30);
        VelConstraint velFast = new TranslationalVelConstraint(45);

        AccelConstraint accSlow = new ProfileAccelConstraint(-30, 30);
        AccelConstraint accFast = new ProfileAccelConstraint(-45, 45);

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
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

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

                drive.updatePoseEstimate();

                TrajectoryActionBuilder t1 = drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(1))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(0,0),velSlow, accSlow)
                        .waitSeconds(0.5);

                Action traj = t1.build();

                TelemetryPacket packet = new TelemetryPacket();


                // updated based on gamepads

                // update running actions
                List<Action> newActions = new ArrayList<>();
                for (Action action : runningActions) {
                    action.preview(packet.fieldOverlay());
                    if (action.run(packet)) {
                        newActions.add(action);
                    }
                }
                runningActions = newActions;

                if(gamepad1.options && !optionsPressed){

                Actions.runBlocking(
                        traj
                );

                }
                optionsPressed = gamepad1.options;

                if(gamepad1.share){
                    drive.pose=new Pose2d(0,0, Math.toRadians(0.5));
                    drive.updatePoseEstimate();
                }


                if (gamepad1.y || gamepad1.triangle) {

                    runningActions.add(
                            new ParallelAction(
                                    elbow.ElbowEject(),
                                    wrist.WristMiddle()
                            )
                    );
                    if(hSlides.getCurrentPosition()<175){
                        runningActions.add(
                                 hslide.HSlideToDist(175)
                        );
                    }
                    runningActions.add(
                            hand.HandOuttake()
                    );
                }


                if ((gamepad1.x || gamepad1.square)&&!gamepadXpressed) {

                    if(SpecimenMode && !outtakeIsOut) {

                        if (specigrabberIsOpen) {
                            runningActions.add(
                                    specigrabber.SpecigrabberClose()
                            );
                            specigrabberIsOpen = false;
                        }else{
                            runningActions.add(
                                    specigrabber.SpecigrabberOpen()
                            );
                            specigrabberIsOpen = true;
                        }

                    } else if (BasketMode && !specigrabberIsOpen && vslides.getCurrentPosition() > 400){
                        if(outtakeIsOut){
                            runningActions.add(
                                    outtake.OuttakeIdle()
                            );
                            outtakeIsOut = false;
                        }else{
                            runningActions.add(
                                    outtake.OuttakeOut()
                            );
                            outtakeIsOut = true;
                            SampleTransferred = false;
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        }
                    }

                }
                gamepadXpressed  = gamepad1.x || gamepad1.square;


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
                if (gamepad1.left_trigger > 0.1 && hSlidesPos < var.hSlideRuleMax) {
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

                    runningActions.add(
                            hand.HandStop()
                    );

                    if(hSlides.getCurrentPosition() > 150) {

                        if (!currentlyIntaking) {  // Ensures prep mode is first if idle
                            intakeMode = false;
                        }
                        currentlyIntaking = true;
                        intakeMode = !intakeMode;  // Toggle intake mode

                        // Adjusts wrist and elbow positions based on intakeMode state
                        if (intakeMode) {
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                            runningActions.add(
                                    new ParallelAction(
                                            wrist.WristIntake(),
                                            elbow.PrepElbowIntake(),
                                            hand.HandIntake()
                                    )
                            );
                        } else {
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                            runningActions.add(
                                    new ParallelAction(
                                            wrist.WristIntake(),
                                            elbow.ElbowIntake(),
                                            hand.HandIntake()
                                    )
                            );
                        }
                    }
                }
                gamepadApressed = gamepad1.a || gamepad1.cross;



                // Transfer action with B button if intake is active
                if ((gamepad1.b || gamepad1.circle) && currentlyIntaking) {
                    currentlyIntaking = false;

                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

                    runningActions.add(
                            new SequentialAction(
                                    new ParallelAction(
                                            wrist.WristOuttake(),
                                            elbow.ElbowOuttake(),
                                            hand.HandStop(),
                                            hslide.HSlideToTransfer(),
                                            vslides.VSlidesTo0()
                                    ),
                                    new SleepAction(1),
                                    hand.HandOuttake()
                            )
                    );

                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);

                    SampleTransferred = true;
                    // Resets hSlides position variable to OuttakePos for consistency
                    hSlidesPos = var.hSlideOuttakePos;

                } else if ((gamepad1.b || gamepad1.circle)) {
                    runningActions.add(
                            hand.HandOuttake() //If you press B and weren't intaking just move the wheels to outtake
                    );
                }



                if (gamepad1.dpad_up) {
                    runningActions.add(
                            hand.HandStop()
                    );
                    if (SpecimenMode){
                        runningActions.add(
                                vslides.VSlidesToDist(var.vSlideHighChamber)
                        );
                        vSlidesPos = var.vSlideHighChamber;
                    } else if (BasketMode) {
                        runningActions.add(
                                vslides.VSlidesToDist(var.vSlideHighBasket)
                        );
                        vSlidesPos = var.vSlideHighBasket;
                    }
                }



                if (gamepad1.dpad_left) {
                    runningActions.add(
                            hand.HandStop()
                    );
                    if (SpecimenMode){
                        runningActions.add(
                                vslides.VSlidesToDist(var.vSlideLowChamber)
                        );
                        vSlidesPos = var.vSlideLowChamber;
                    } else if (BasketMode) {
                        runningActions.add(
                                vslides.VSlidesToDist(var.vSlideLowBasket)
                        );
                        vSlidesPos = var.vSlideLowBasket;
                    }
                }



                if (gamepad1.dpad_right && !dPadRightPressed) {
                    if (SpecimenMode && !outtakeIsOut && (vslides.getCurrentPosition() < var.vSlideHighChamber + 30 && vslides.getCurrentPosition() > var.vSlideHighChamber - 30)) {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                        runningActions.add(
                                new SequentialAction(
                                        vslides.VSlidesToDist(var.vSlideHighChamberDrop, 300),
                                        specigrabber.SpecigrabberOpen()
                                )
                        );
                        vSlidesPos=var.vSlideHighChamberDrop;
                        specigrabberIsOpen = true;
                    } else if (SpecimenMode && !outtakeIsOut && (vslides.getCurrentPosition() < var.vSlideLowChamber + 30 && vslides.getCurrentPosition() > var.vSlideLowChamber - 30)) {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                        runningActions.add(
                                new SequentialAction(
                                        vslides.VSlidesToDist(var.vSlideLowChamberDrop, 300),
                                        specigrabber.SpecigrabberOpen()
                                )
                        );
                        vSlidesPos=var.vSlideLowChamberDrop;
                        specigrabberIsOpen = true;
                    } else if(BasketMode && !specigrabberIsOpen && vslides.getCurrentPosition() > 400 && SampleTransferred){
                        runningActions.add(
                                outtake.OuttakeOut()
                        );
                        outtakeIsOut = true;
                        SampleTransferred = false;
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    } else if(BasketMode && !specigrabberIsOpen && vslides.getCurrentPosition() > 400 && !SampleTransferred){
                        runningActions.add(
                                new ParallelAction(
                                        outtake.OuttakeIdle(),
                                        vslides.VSlidesTo0()
                                )
                        );
                        vSlidesPos = 0;
                        outtakeIsOut = false;
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    }
                }
                dPadRightPressed = gamepad1.dpad_right;



                if (gamepad1.dpad_down && vslides.getCurrentPosition() > 50) {

                    runningActions.add(
                            new ParallelAction(
                                    vslides.VSlidesTo0(),
                                    outtake.OuttakeIdle()
                            )
                        );

                } else if(gamepad1.dpad_down && !dPadDownPressed){
                    runningActions.add(
                            hand.HandStop()
                    );

                    if(hSlides.getCurrentPosition() > 225) {

                        currentlyIntaking = true;
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);

                            runningActions.add(
                                    new ParallelAction(
                                            wrist.WristToDist(var.FrontIntakeWristPos),
                                            elbow.ElbowToDist(var.FrontIntakeElbowPos),
                                            hand.HandIntake()
                                    )
                            );

                    }

                }
                dPadDownPressed = gamepad1.dpad_down;



                if (gamepad1.left_stick_button){
                    BasketMode = false;
                    SpecimenMode = true;
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                }

                if (gamepad1.right_stick_button){
                    BasketMode = true;
                    SpecimenMode = false;
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }



                //Slower when intaking or when stuff is extended
                if(hSlides.getCurrentPosition()>250||lArm.getCurrentPosition()>1400||rArm.getCurrentPosition()>1400||currentlyIntaking){
                    driveSpeed = 2.5;
                } else{
                    driveSpeed = 1.5;
                }



                //Failsafes
                if(vslides.getCurrentPosition()<400&& outtakeIsOut && !specigrabberIsOpen){
                    runningActions.add(
                            outtake.OuttakeIdle()
                    );
                }

                if(currentlyIntaking && hSlidesPos<150){
                    hSlidesPos = 150;
                }

                if(vSlidesPos<0){
                    vSlidesPos = 0;
                } else if (vSlidesPos>var.vSlidePhysicalMax) {
                    vSlidesPos = var.vSlidePhysicalMax;
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
                telemetry.addData("VSlidesPosVariable = ", vSlidesPos);
                telemetry.addData("currentHorizontalSlidesPos = ", hSlides.getCurrentPosition());
                telemetry.addData("HSlidesPosVariable = ", hSlidesPos);
                telemetry.addData("IntakeMode = ", intakeMode);
                telemetry.addData("Currently Intaking: = ", currentlyIntaking);
                telemetry.addData("Transfer: = ", SampleTransferred);
                telemetry.update();

                dash.sendTelemetryPacket(packet);

            }
        }
    }
}