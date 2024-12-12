package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.LEAGUE_MEET_2.TeleOp;


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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Elbow;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Hand;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Specigrabber;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Wrist;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM2_SUBSYSTEMS.OuttakeLM2;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name = "LM2 - 1 Driver Comp Tele-Op V1", group = "TeleOp")
public class LM2_CompTeleOp1Controller_V1 extends LinearOpMode {

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

        DigitalChannel pin0 = hardwareMap.digitalChannel.get("digital0");
        DigitalChannel pin1 = hardwareMap.digitalChannel.get("digital1");

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
        OuttakeLM2 outtakeLM2 = new OuttakeLM2(hardwareMap);
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
        TouchSensor vTouchLeft = hardwareMap.get(TouchSensor.class, "vTouchLeft");
        TouchSensor vTouchRight = hardwareMap.get(TouchSensor.class, "vTouchRight");

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
        boolean dPadLeftPressed = false; // Tracks DpadRight button state
        boolean dPadUpPressed = false; // Tracks DpadRight button state
        boolean optionsPressed = false;
        boolean leftStickPressed = false;
        boolean rightStickPressed = false;

        boolean YELLOW_DETECTED = false;
        boolean BLUE_DETECTED = false;
        boolean RED_DETECTED = false;

        int vSlidesPos = vslides.getCurrentPosition(); // Variable for vertical slides position
        int hSlidesPos = hSlides.getCurrentPosition(); // Variable for horizontal slides position

        boolean SpecimenMode = false;
        boolean BasketMode = true;

        boolean outtakeIsOut = false;
        boolean specigrabberIsOpen = false;

        int debugModeSet = 0;
        boolean debugModeIsOn = false;

        VelConstraint velSlow = new TranslationalVelConstraint(30);
        VelConstraint velFast = new TranslationalVelConstraint(45);

        AccelConstraint accSlow = new ProfileAccelConstraint(-30, 30);
        AccelConstraint accFast = new ProfileAccelConstraint(-45, 45);


        // Set initial LED pattern
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        // Initialize servos to default positions using ParallelAction




        // Wait for start button to be pressed
        waitForStart();

        if (opModeIsActive()) {



            while ((gamepad1.left_stick_x+gamepad1.left_stick_y+gamepad1.right_stick_x+gamepad1.right_stick_y==0)){
                telemetry.addLine("WAITING FOR START");
                telemetry.update();
            }

            // Set horizontal slides initial position
            hSlides.setTargetPosition(hSlidesPos);
            hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) hSlides).setVelocity(var.hSlideVelocity);

            // Set vertical slides initial position
            lArm.setTargetPosition(vSlidesPos);
            rArm.setTargetPosition(vSlidesPos);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            Actions.runBlocking(
                    new ParallelAction(
                            wrist.WristMiddle(),
                            elbow.ElbowMiddle(),
                            specigrabber.SpecigrabberClose(),
                            outtakeLM2.OuttakeIdle()
                    )
            );



            while (opModeIsActive()) {


                if(pin0.getState()&&pin1.getState()){
                    YELLOW_DETECTED = true;
                    RED_DETECTED = false;
                    BLUE_DETECTED = false;
                } else if(pin0.getState()&&!pin1.getState()){
                    YELLOW_DETECTED = false;
                    RED_DETECTED = false;
                    BLUE_DETECTED = true;
                }else if(pin1.getState()&&!pin0.getState()){
                    YELLOW_DETECTED = false;
                    RED_DETECTED = true;
                    BLUE_DETECTED = false;
                }else{
                    YELLOW_DETECTED = false;
                    RED_DETECTED = true;
                    BLUE_DETECTED = false;
                }


                if(vslides.getCurrentPosition()>250){
                    hSlidesPos = 150;
                    hSlides.setTargetPosition(hSlidesPos);
                    hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) hSlides).setVelocity(var.hSlideVelocity);
                }



                if(vTouchLeft.isPressed()){
                    lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if(vTouchRight.isPressed()){
                    rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }


                //For roadrunner localization during tele-op
                drive.updatePoseEstimate();

                TrajectoryActionBuilder t1 = drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(1))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(0,0),velSlow, accSlow)
                        .waitSeconds(0.5);

                Action traj = t1.build();

                TelemetryPacket packet = new TelemetryPacket();

                List<Action> newActions = new ArrayList<>();
                for (Action action : runningActions) {
                    action.preview(packet.fieldOverlay());
                    if (action.run(packet)) {
                        newActions.add(action);
                    }
                }
                runningActions = newActions;

                if((gamepad1.options) && !optionsPressed){

                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);

                    Actions.runBlocking(
                            traj
                    );

                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

                }
                optionsPressed = gamepad1.options;

                if(gamepad1.share){
                    drive.pose=new Pose2d(0,0, Math.toRadians(0.5));
                    drive.updatePoseEstimate();
                }


                //Above is for making the robot move to a position automatically



                //EJECT BUTTON
                if ((gamepad1.y || gamepad1.triangle)&&!debugModeIsOn) {

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
                        hSlidesPos=175;
                    }
                    runningActions.add(
                            hand.HandOuttake()
                    );
                } else if ((gamepad1.y || gamepad1.triangle)&&debugModeIsOn) {
                    runningActions.add(
                            new ParallelAction(
                                    wrist.WristDebug(),
                                    elbow.ElbowDebugUp()
                            )
                    );
                }

                if((gamepad1.x||gamepad1.square)&&!gamepadXpressed&&debugModeIsOn){
                    runningActions.add(
                            new ParallelAction(
                                    wrist.WristDebug(),
                                    elbow.ElbowDebugDown()
                            )
                    );
                }

                if((gamepad1.a || gamepad1.cross) && !gamepadApressed && debugModeIsOn){
                    runningActions.add(
                            hand.HandIntake()
                    );
                }



                //CLAW AND OUTTAKE OPEN AND CLOSE BUTTON
                if ((gamepad1.x || gamepad1.square)&&!gamepadXpressed && !debugModeIsOn) {

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
                                    outtakeLM2.OuttakeIdle()
                            );
                            outtakeIsOut = false;
                        }else{
                            runningActions.add(
                                    outtakeLM2.OuttakeOut()
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
                if (gamepad1.right_bumper && vSlidesPos < var.vSlidePhysicalMax &&!debugModeIsOn) {
                    vSlidesPos += 25;
                    lArm.setTargetPosition(vSlidesPos);
                    rArm.setTargetPosition(vSlidesPos);
                    lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                    ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                }

                // Moves slides down when right bumper is pressed, but stops at minimum height
                if (gamepad1.left_bumper && vSlidesPos > 0 &&!debugModeIsOn) {
                    vSlidesPos -= 25;
                    lArm.setTargetPosition(vSlidesPos);
                    rArm.setTargetPosition(vSlidesPos);
                    lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                    ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                }



                //DEBUG VERSIONS
                if (gamepad1.right_bumper && debugModeIsOn) {
                    vSlidesPos += 15;
                    lArm.setTargetPosition(vSlidesPos);
                    rArm.setTargetPosition(vSlidesPos);
                    lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                    ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                }
                if (gamepad1.left_bumper && debugModeIsOn) {
                    vSlidesPos -= 15;
                    lArm.setTargetPosition(vSlidesPos);
                    rArm.setTargetPosition(vSlidesPos);
                    lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                    ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                }





                // Horizontal slides control - moves slides outward with left trigger,
                // increases speed with harder press, but stops at max position
                if (gamepad1.left_trigger > 0.1 && hSlidesPos < var.hSlideRuleMax && !debugModeIsOn) {
                    hSlidesPos += (int) (10 * (gamepad1.left_trigger));
                    hSlides.setTargetPosition(hSlidesPos);
                    hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) hSlides).setVelocity(var.hSlideVelocity);
                }

                // Moves slides inward with right trigger, but stops at minimum position
                if (gamepad1.right_trigger > 0.1 && hSlidesPos > var.hSlideTransferPos && !debugModeIsOn) {
                    hSlidesPos -= (int) (10 * (gamepad1.right_trigger));
                    hSlides.setTargetPosition(hSlidesPos);
                    hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) hSlides).setVelocity(var.hSlideVelocity);
                }

                if (gamepad1.left_trigger > 0.1 && debugModeIsOn) {
                    hSlidesPos += (int) (8 * (gamepad1.left_trigger));
                    hSlides.setTargetPosition(hSlidesPos);
                    hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) hSlides).setVelocity(var.hSlideVelocity);
                }

                // Moves slides inward with right trigger, but stops at minimum position
                if (gamepad1.right_trigger > 0.1 && debugModeIsOn) {
                    hSlidesPos -= (int) (8 * (gamepad1.right_trigger));
                    hSlides.setTargetPosition(hSlidesPos);
                    hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) hSlides).setVelocity(var.hSlideVelocity);
                }


                // Intake mode toggle with A button
                if ((gamepad1.a || gamepad1.cross) && !gamepadApressed &&!debugModeIsOn) {  // When A button is newly pressed

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

                    hSlidesPos = var.hSlideTransferPos;

                    hSlides.setTargetPosition(hSlidesPos);
                    hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) hSlides).setVelocity(var.hSlideVelocity);

                    runningActions.add(
                            new SequentialAction(
                                    new ParallelAction(
                                            wrist.WristTransfer(),
                                            elbow.ElbowTransfer(),
                                            hand.HandStop()
                                    ),
                                    new SleepAction(0.75),
                                    hand.HandOuttake()
                            )
                    );

                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);

                    SampleTransferred = true;
                    // Resets hSlides position variable to OuttakePos for consistency

                }



                if (gamepad1.dpad_up && !dPadUpPressed) {
                    runningActions.add(
                            hand.HandStop()
                    );
                    if (SpecimenMode){
                        vSlidesPos = var.vSlideHighChamber;
                        lArm.setTargetPosition(vSlidesPos);
                        rArm.setTargetPosition(vSlidesPos);
                        lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                        ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                    } else if (BasketMode) {
                        vSlidesPos = var.vSlideHighBasket;
                        lArm.setTargetPosition(vSlidesPos);
                        rArm.setTargetPosition(vSlidesPos);
                        lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                        ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                    }
                }
                dPadUpPressed = gamepad1.dpad_up;




                if (gamepad1.dpad_left && !dPadLeftPressed) {
                    runningActions.add(
                            hand.HandStop()
                    );
                    if (SpecimenMode){
                        vSlidesPos = var.vSlideLowChamber;
                        lArm.setTargetPosition(vSlidesPos);
                        rArm.setTargetPosition(vSlidesPos);
                        lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                        ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                    } else if (BasketMode) {
                        vSlidesPos = var.vSlideLowBasket;
                        lArm.setTargetPosition(vSlidesPos);
                        rArm.setTargetPosition(vSlidesPos);
                        lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                        ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                    }
                }
                dPadLeftPressed = gamepad1.dpad_left;




                if (gamepad1.dpad_right && !dPadRightPressed) {
                    if (SpecimenMode && !outtakeIsOut && (vslides.getCurrentPosition() < var.vSlideHighChamber + 30 && vslides.getCurrentPosition() > var.vSlideHighChamber - 30)) {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                        runningActions.add(
                                new SequentialAction(
                                        vslides.VSlidesToDist(var.vSlideHighChamberDrop, 250),
                                        new SleepAction(1.5),
                                        specigrabber.SpecigrabberOpen()
                                )
                        );
                        vSlidesPos=var.vSlideHighChamberDrop;
                        specigrabberIsOpen = true;
                    } else if (SpecimenMode && !outtakeIsOut && (vslides.getCurrentPosition() < var.vSlideLowChamber + 30 && vslides.getCurrentPosition() > var.vSlideLowChamber - 30)) {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                        runningActions.add(
                                new SequentialAction(
                                        vslides.VSlidesToDist(var.vSlideLowChamberDrop, 250),
                                        new SleepAction(1.5),
                                        specigrabber.SpecigrabberOpen()
                                )
                        );
                        vSlidesPos=var.vSlideLowChamberDrop;
                        specigrabberIsOpen = true;
                    } else if(BasketMode && !specigrabberIsOpen && vslides.getCurrentPosition() > 400 && SampleTransferred){
                        runningActions.add(
                                outtakeLM2.OuttakeOut()
                        );
                        outtakeIsOut = true;
                        SampleTransferred = false;
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    } else if(BasketMode && !specigrabberIsOpen && vslides.getCurrentPosition() > 400 && !SampleTransferred){
                        runningActions.add(
                                new ParallelAction(
                                        outtakeLM2.OuttakeIdle(),
                                        new SleepAction(1.25),
                                        vslides.VSlidesTo0()
                                )
                        );
                        vSlidesPos = 0;
                        outtakeIsOut = false;
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    }
                }
                dPadRightPressed = gamepad1.dpad_right;



                if (gamepad1.dpad_down && vslides.getCurrentPosition() > 50 && !dPadDownPressed) {
                    vSlidesPos = 0;
                    lArm.setTargetPosition(vSlidesPos);
                    rArm.setTargetPosition(vSlidesPos);
                    lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) lArm).setVelocity(var.vSlideVelocity);
                    ((DcMotorEx) rArm).setVelocity(var.vSlideVelocity);
                    runningActions.add(
                            new ParallelAction(
                                    outtakeLM2.OuttakeIdle()
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



                if (gamepad1.left_stick_button&&!debugModeIsOn&&!gamepad1.right_stick_button){
                    BasketMode = false;
                    SpecimenMode = true;
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    runningActions.add(
                            outtakeLM2.OuttakeIdle()
                    );
                    outtakeIsOut = false;
                }

                if (gamepad1.right_stick_button&&!debugModeIsOn&&!gamepad1.left_stick_button){
                    BasketMode = true;
                    SpecimenMode = false;
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    runningActions.add(
                            specigrabber.SpecigrabberClose()
                    );
                    specigrabberIsOpen = false;
                }

                if (gamepad1.left_stick_button&&debugModeIsOn&&!leftStickPressed&&!gamepad1.right_stick_button){
                    lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
                    vSlidesPos = 0;
                }

                if (gamepad1.right_stick_button&&debugModeIsOn&&!rightStickPressed&&!gamepad1.left_stick_button){
                    hSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_GRAY);
                    hSlidesPos = 0;
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
                            outtakeLM2.OuttakeIdle()
                    );
                }

                if(currentlyIntaking && hSlidesPos<150 && !debugModeIsOn){
                    hSlidesPos = 150;
                }

                if(vSlidesPos<0 && !debugModeIsOn){
                    vSlidesPos = 0;
                } else if (vSlidesPos>var.vSlidePhysicalMax && !debugModeIsOn) {
                    vSlidesPos = var.vSlidePhysicalMax;
                }

                if(gamepad1.left_stick_button&&gamepad1.right_stick_button&&!debugModeIsOn){
                    debugModeSet+=1;
                    if (debugModeSet >= 45){
                        debugModeIsOn = true;
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
                        debugModeSet = 0;
                    }
                }
                if(gamepad1.left_stick_button&&gamepad1.right_stick_button&&debugModeIsOn){
                    debugModeSet += 1;
                    if (debugModeSet >= 45) {
                        debugModeIsOn = false;
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        debugModeSet=0;
                    }
                }
                leftStickPressed = gamepad1.left_stick_button;
                rightStickPressed = gamepad1.right_stick_button;


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
                telemetry.addData("BasketMode", BasketMode);
                telemetry.addData("IntakeMode = ", intakeMode);
                telemetry.addData("Currently Intaking: = ", currentlyIntaking);
                telemetry.addData("Transfer: = ", SampleTransferred);
                telemetry.addData("DebugModeSet: = ", debugModeSet);
                telemetry.addData("DebugMode: = ", debugModeIsOn);
                telemetry.addData("vTouchLeft: = ", vTouchLeft.isPressed());
                telemetry.addData("vTouchRight: = ", vTouchRight.isPressed());
                telemetry.addData("YELLOW DETECTED", pin0.getState()&&pin1.getState());
                telemetry.addData("BLUE DETECTED", pin0.getState()&&!pin1.getState());
                telemetry.addData("RED DETECTED", pin1.getState()&&!pin0.getState());
                telemetry.update();

                dash.sendTelemetryPacket(packet);

            }
        }
    }
}