package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.LEAGUE_TOURNAMENT.TeleOp;


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
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Elbow;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Wrist;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM2_SUBSYSTEMS.OuttakeLM2;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM3_SUBSYSTEMS.HandLM3;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM3_SUBSYSTEMS.SpecigrabberLM3;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@TeleOp(name = "LMT - 2 Drivers Comp TeleOp V1", group = "TeleOp")
public class LMT_CompTeleOp2Controllers extends LinearOpMode {

    /*

                         READ ME

    BEFORE EDITING THIS TELE-OP, MAKE SURE TO READ THE
             " INTO THE DEEP TELE-OP CONTROLS "
    DRIVE FILE IN THE GOOGLE DRIVE, THIS LISTS WHAT WE SHOULD
    HAVE EACH BUTTON DO TO OPTIMIZE THE TELE-OP PERIOD.

     */


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
        HandLM3 handLM3 = new HandLM3(hardwareMap);
        OuttakeLM2 outtakeLM2 = new OuttakeLM2(hardwareMap);
        SpecigrabberLM3 specigrabber = new SpecigrabberLM3(hardwareMap);

        // 'var' holds subsystem-specific variables such as physical limits and velocities
        // for easier access and adjustment across the tele-op program
        SubsystemsVariables var = new SubsystemsVariables();

        // Hardware initialization
        // Motor declarations
        CachingDcMotorEx leftFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        CachingDcMotorEx leftBack = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftBack"));
        CachingDcMotorEx lArm = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "lArm"));
        CachingDcMotorEx hSlides = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "hSlides"));
        CachingDcMotorEx rArm = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rArm"));
        CachingDcMotorEx rightBack = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightBack"));
        CachingDcMotorEx rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));
        DcMotorEx specimenArm = hardwareMap.get(DcMotorEx.class, "specimenArm");

        CachingServo PTOleft = new CachingServo(hardwareMap.get(Servo.class, "PTOleft"));
        CachingServo PTOright = new CachingServo(hardwareMap.get(Servo.class, "PTOright"));

        lArm.setCachingTolerance(0.001);
        rArm.setCachingTolerance(0.001);

        RevBlinkinLedDriver lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
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

        boolean frontIntakeActive = false;
        boolean hSlideTransferCheck = false;

        boolean ejectStage = false;


        boolean specimenArmHasPid = true;
        boolean vSlidesHavePid = true;

        //specimenArm.setCurrentAlert(3.5, CurrentUnit.AMPS);

        int specimenGrabOffset = var.speciArmGrab+30;

        boolean gamepadApressed = false; // Tracks A button state
        boolean gamepadXpressed = false; // Tracks X button state
        boolean gamepadBPressed = false; // Tracks B button state
        boolean gamepadYpressed = false; // Tracks Y button state
        boolean dPadDownPressed = false; // Tracks DpadDpwn button state
        boolean dPadRightPressed = false; // Tracks DpadRight button state
        boolean dPadLeftPressed = false; // Tracks DpadRight button state
        boolean dPadUpPressed = false; // Tracks DpadRight button state
        boolean optionsPressed = false; // Tracks options pressed button state
        boolean leftStickPressed = false; // same as ^^^
        boolean rightStickPressed = false; // same as ^^^
        //All the "button state" variables are to make sure when we press
        //a button it doesn't do the action over and over after each loop, just once

        boolean hSlideMoved = false; //tracks whether the hslide was moved already by
        //the vslides being extended
        boolean lightsChanged = false; //tracks whether the sample sensor made the lights change alr


        boolean YELLOW_DETECTED = false;
        boolean BLUE_DETECTED = false;
        boolean RED_DETECTED = false;

        int vSlidesPos = vslides.getCurrentPosition(); // Variable for vertical slides position
        int hSlidesPos = hSlides.getCurrentPosition(); // Variable for horizontal slides position
        int specArmPos = specimenArm.getCurrentPosition(); // Variable for horizontal slides position
        int colorDetectionThreshold = 0;

        boolean SpecimenMode = false;
        boolean BasketMode = true;

        boolean outtakeIsOut = false;
        boolean specigrabberIsOpen = false;

        int debugModeSet = 0;
        boolean debugModeIsOn = false;

        int lArmCurrentPos;
        int rArmCurrentPos;
        int vSlidesCurrentPos = vslides.getCurrentPosition(); // Variable for vertical slides position
        int hSlidesCurrentPos = hSlides.getCurrentPosition(); // Variable for horizontal slides position

        VelConstraint velSlow = new TranslationalVelConstraint(30);
        VelConstraint velFast = new TranslationalVelConstraint(45);

        AccelConstraint accSlow = new ProfileAccelConstraint(-30, 30);
        AccelConstraint accFast = new ProfileAccelConstraint(-45, 45);


        // Set initial LED pattern
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);


        //PID STUFF
        PIDController controller;

        double p = 0.0065, i = 0, d = 0.000001;
        double f = 0.00007;
        double relativeP = 0.003;

        double leftPower;
        double rightPower;

        double motorRelativeError;
        double power;
        double denom;

        PIDController controllerSpec;

        double pidSPEC;
        double ffSPEC;

        ElapsedTime elapsedTime;

        // Wait for start button to be pressed
        waitForStart();

        if (opModeIsActive()) {

            elapsedTime = new ElapsedTime();
            elapsedTime.reset();

            controller = new PIDController(p, i, d);
            controller.setPIDF(p, i, d, f);

            controllerSpec = new PIDController(.005, .075, .0002);

            

            while ((gamepad1.left_stick_x+gamepad1.left_stick_y+gamepad1.right_stick_x+gamepad1.right_stick_y==0)){
                telemetry.addLine("WAITING FOR START");
                telemetry.addData("Loop Times", elapsedTime.milliseconds());
                elapsedTime.reset();
                telemetry.update();
            }

            // Set horizontal slides initial position
            hSlides.setTargetPosition(hSlidesPos);
            hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hSlides.setVelocity(var.hSlideVelocity);

            // Set vertical slides initial position
            lArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            lArmCurrentPos = lArm.getCurrentPosition();
            rArmCurrentPos = rArm.getCurrentPosition();

            //specimenArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            specimenArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            specimenArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            Actions.runBlocking(
                    new ParallelAction(
                            wrist.WristMiddle(),
                            elbow.ElbowMiddle(),
                            specigrabber.SpecigrabberClose(),
                            outtakeLM2.OuttakeIdle()
                    )
            );



            while (opModeIsActive()) {

                lArmCurrentPos = lArm.getCurrentPosition();
                rArmCurrentPos = rArm.getCurrentPosition();
                vSlidesCurrentPos = vslides.getCurrentPosition();
                hSlidesCurrentPos = hSlides.getCurrentPosition();


                telemetry.addData("Loop Times", elapsedTime.milliseconds());
                elapsedTime.reset();


                //PID CONTROLLER FOR VERTICAL SLIDES
                motorRelativeError = Math.abs(lArmCurrentPos- rArmCurrentPos)>1? lArmCurrentPos- rArmCurrentPos:0;
                power = controller.calculate((lArmCurrentPos+ rArmCurrentPos)/2, vSlidesPos);

                leftPower = power-relativeP*motorRelativeError;
                rightPower = power+relativeP*motorRelativeError;
                denom = Math.max(leftPower, Math.max(rightPower, 1));

                //SET POWER BASED ON VSLIDES POS VARIABLE

                    lArm.setPower(leftPower / denom);
                    rArm.setPower(rightPower / denom);


                //PID FOR SPEC ARM

                controllerSpec.setPID(.005, .075, .0002);
                pidSPEC = controllerSpec.calculate(specimenArm.getCurrentPosition(), specArmPos);
                ffSPEC = Math.cos(Math.toRadians(specArmPos / ((2786.2/360)/2))) * .05;

                if(specimenArmHasPid) {
                    specimenArm.setPower(pidSPEC + ffSPEC);
                }

                    hSlides.setTargetPosition(hSlidesPos);
                    hSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hSlides.setVelocity(var.hSlideVelocity);



                if(pin0.getState()&&pin1.getState()){
                    colorDetectionThreshold++;
                    if(currentlyIntaking&&!lightsChanged&&colorDetectionThreshold>15){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                        lightsChanged = true;
                    }
                    YELLOW_DETECTED = true;
                    RED_DETECTED = false;
                    BLUE_DETECTED = false;
                } else if(pin0.getState()&&!pin1.getState()){
                    colorDetectionThreshold++;
                    if(currentlyIntaking&&!lightsChanged&&colorDetectionThreshold>15){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                        lightsChanged = true;
                    }
                    YELLOW_DETECTED = false;
                    RED_DETECTED = false;
                    BLUE_DETECTED = true;
                }else if(pin1.getState()&&!pin0.getState()){
                    colorDetectionThreshold++;
                    if(currentlyIntaking&&!lightsChanged&&colorDetectionThreshold>15){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        lightsChanged = true;
                    }
                    YELLOW_DETECTED = false;
                    RED_DETECTED = true;
                    BLUE_DETECTED = false;
                }else{
                    colorDetectionThreshold = 0;
                    lightsChanged = false;
                    YELLOW_DETECTED = false;
                    RED_DETECTED = true;
                    BLUE_DETECTED = false;
                }



                if(vSlidesCurrentPos>250&&!hSlideMoved){
                    hSlidesPos = 150;
                    hSlideMoved = true;
                } else if (vSlidesCurrentPos<=250) {
                    hSlideMoved = false;
                }


                if(vTouchLeft.isPressed()){
                    lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                }
                if(vTouchRight.isPressed()){
                    rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                }


                //For roadrunner localization during tele-op
                drive.updatePoseEstimate();

                TrajectoryActionBuilder t1 = drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(1))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(0,0),velFast, accFast)
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



                if((gamepad2.options||gamepad1.options) && !optionsPressed){

                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);

                Actions.runBlocking(
                        traj
                );

                if(BasketMode) {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }else{
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                }

                }
                optionsPressed = gamepad2.options||gamepad1.options;

                if(gamepad1.share){
                    drive.pose=new Pose2d(0,0, Math.toRadians(0.5));
                    drive.updatePoseEstimate();
                }


                //Above is for making the robot move to a position automatically



                //EJECT BUTTON
                if ((gamepad1.y || gamepad1.triangle)&&!debugModeIsOn &&!gamepadYpressed && !ejectStage) {
                    intakeMode = false;
                    ejectStage = true;

                    runningActions.add(
                            new ParallelAction(
                                    elbow.ElbowEject(),
                                    wrist.WristMiddle(),
                                    handLM3.HandStop()
                            )
                    );
                    if(hSlidesCurrentPos<135){
                        hSlidesPos=135;
                    }
                } else if ((gamepad1.y || gamepad1.triangle)&&!debugModeIsOn &&!gamepadYpressed && ejectStage) {
                    runningActions.add(
                            handLM3.HandOuttake()
                    );
                    ejectStage= false;
                } else if ((gamepad1.y || gamepad1.triangle)&&debugModeIsOn &&!gamepadYpressed) {
                    runningActions.add(
                            new ParallelAction(
                                    wrist.WristDebug(),
                                    elbow.ElbowDebugUp()
                            )
                    );
                }
                gamepadYpressed = gamepad1.y || gamepad1.triangle;

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
                                    handLM3.HandIntake()
                    );
                }



                //CLAW AND OUTTAKE OPEN AND CLOSE BUTTON
                if ((gamepad2.x || gamepad2.square)&&!gamepadXpressed && !debugModeIsOn) {

                    if(SpecimenMode) {

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

                    } else if (BasketMode && vSlidesCurrentPos > 400){
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
                gamepadXpressed  = gamepad2.x || gamepad2.square || gamepad1.x || gamepad1.square;




                // Vertical slide control - moves slides up when left bumper is pressed,
                // but not past the maximum allowed height
                if (gamepad2.right_bumper && vSlidesPos < var.vSlidePhysicalMax &&!debugModeIsOn) {
                    vSlidesPos += 25;

                }

                // Moves slides down when right bumper is pressed, but stops at minimum height
                if (gamepad2.left_bumper && vSlidesPos > 0 &&!debugModeIsOn) {
                    vSlidesPos -= 25;

                }



                //DEBUG VERSIONS
                if (gamepad2.right_bumper && debugModeIsOn) {
                    vSlidesPos += 15;

                }
                if (gamepad1.left_bumper && debugModeIsOn) {
                    vSlidesPos -= 15;
                }





                // Horizontal slides control - moves slides outward with left trigger,
                // increases speed with harder press, but stops at max position
                if (gamepad1.right_trigger > 0.1 && hSlidesPos < var.hSlideRuleMax && !debugModeIsOn) {
                    hSlidesPos += (int) (17 * (gamepad1.right_trigger));
                }

                // Moves slides inward with right trigger, but stops at minimum position
                if (gamepad1.left_trigger > 0.1 && hSlidesPos > var.hSlideTransferPos && !debugModeIsOn) {
                    hSlidesPos -= (int) (17 * (gamepad1.left_trigger));
                }

                if (gamepad1.right_trigger > 0.1 && debugModeIsOn) {
                    hSlidesPos += (int) (10 * (gamepad1.right_trigger));
                }

                // Moves slides inward with right trigger, but stops at minimum position
                if (gamepad1.left_trigger > 0.1 && debugModeIsOn) {
                    hSlidesPos -= (int) (10 * (gamepad1.left_trigger));
                }


                // Intake mode toggle with A button
                if ((gamepad1.a || gamepad1.cross) && !gamepadApressed &&!debugModeIsOn) {  // When A button is newly pressed

                    runningActions.add(
                            handLM3.HandStop()
                    );

                    if(hSlidesCurrentPos > 150) {

                        if (!currentlyIntaking) {  // Ensures prep mode is first if idle
                            intakeMode = false;
                        }
                        frontIntakeActive = false;
                        currentlyIntaking = true;
                        ejectStage = false;
                        intakeMode = !intakeMode;  // Toggle intake mode

                        // Adjusts wrist and elbow positions based on intakeMode state
                        if (intakeMode) {
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
                            runningActions.add(
                                    new ParallelAction(
                                            wrist.WristIntake(),
                                            elbow.PrepElbowIntake(),
                                            handLM3.HandIntake()
                                    )
                            );
                        } else {
                            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                            runningActions.add(
                                    new ParallelAction(
                                            wrist.WristIntake(),
                                            elbow.ElbowIntake(),
                                            handLM3.HandIntake()
                                    )
                            );
                        }
                    }
                }
                gamepadApressed = gamepad1.a || gamepad1.cross;



                // Transfer action with B button if intake is active
                if ((gamepad1.b || gamepad1.circle) && currentlyIntaking && BasketMode && !frontIntakeActive && !gamepadBPressed) {
                    currentlyIntaking = false;
                    frontIntakeActive = false;

                    if(YELLOW_DETECTED){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    }else if(RED_DETECTED){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
                    } else if (BLUE_DETECTED) {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                    }else {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
                    }

                    hSlidesPos = var.hSlideTransferPos;


                    runningActions.add(
                            new SequentialAction(
                                    new ParallelAction(
                                            wrist.WristTransfer(),
                                            elbow.ElbowTransfer(),
                                            handLM3.HandStop()
                                    ),
                                    new SleepAction(0.75),
                                    handLM3.HandOuttake()
                            )
                    );



                    if(YELLOW_DETECTED){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    }else if(RED_DETECTED){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
                    } else if (BLUE_DETECTED) {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                    }else {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    }

                    SampleTransferred = true;

                } else if ((gamepad1.b || gamepad1.circle) && currentlyIntaking && BasketMode && frontIntakeActive && !hSlideTransferCheck && !gamepadBPressed) {
                    currentlyIntaking = false;
                    hSlideTransferCheck = true;
                    frontIntakeActive = false;

                    if(YELLOW_DETECTED){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    }else if(RED_DETECTED){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
                    } else if (BLUE_DETECTED) {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                    }else {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
                    }

                    hSlidesPos = var.hSlideTransferPos+120;

                    runningActions.add(
                            new SequentialAction(
                                    new ParallelAction(
                                            wrist.WristTransfer(),
                                            elbow.ElbowTransfer(),
                                            handLM3.HandStop()
                                    )
                            )
                    );


                    if(YELLOW_DETECTED){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    }else if(RED_DETECTED){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
                    } else if (BLUE_DETECTED) {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                    }else {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    }


                } else if ((gamepad1.b || gamepad1.circle) && BasketMode && hSlideTransferCheck && !gamepadBPressed) {
                    currentlyIntaking = false;

                    hSlideTransferCheck = false;

                    hSlidesPos = var.hSlideTransferPos;

                    runningActions.add(
                            new SequentialAction(
                                    new SleepAction(0.25),
                                    handLM3.HandOuttake()
                            )
                    );

                    SampleTransferred = true;

                } else if ((gamepad1.b || gamepad1.circle) && SpecimenMode && !gamepadBPressed) {

                    //SPECIMEN GRAB EJECT

                    currentlyIntaking = false;
                    frontIntakeActive = false;

                    if(YELLOW_DETECTED){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    }else if(RED_DETECTED){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
                    } else if (BLUE_DETECTED) {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                    }else {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
                    }

                    hSlidesPos = 60;

                    runningActions.add(
                                    new ParallelAction(
                                            wrist.WristMiddle(),
                                            elbow.ElbowEject(),
                                            handLM3.HandStop()
                                    )
                    );

                    if(YELLOW_DETECTED){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    }else if(RED_DETECTED){
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
                    } else if (BLUE_DETECTED) {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                    }else {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    }

                    SampleTransferred = true;
                }
                gamepadBPressed = gamepad1.b || gamepad1.circle;


                if (gamepad2.dpad_up && !dPadUpPressed) {

                    runningActions.add(
                            handLM3.HandStop()
                    );
                    if (SpecimenMode){
                        specArmPos = var.speciArmPrepScore;
                        runningActions.add(
                                new SequentialAction(
                                        specigrabber.SpeciRotateScore()
                                )
                        );
                    } else if (BasketMode) {
                        vSlidesPos = var.vSlideHighBasket;
                    }
                }
                dPadUpPressed = gamepad2.dpad_up;




                if (gamepad2.dpad_left && !dPadLeftPressed) {

                    runningActions.add(
                            handLM3.HandStop()
                    );
                    if (SpecimenMode){
                        specimenArmHasPid = false;
                        specimenArm.setPower(-0.4);
                        runningActions.add(
                                new SequentialAction(
                                        specigrabber.SpeciRotateGrab(),
                                        specigrabber.SpecigrabberOpen()
                                )
                        );
                    } else if (BasketMode) {
                        vSlidesPos = var.vSlideLowBasket-200;
                    }
                }
                dPadLeftPressed = gamepad2.dpad_left;


                if(specimenArmHasPid == false){
                    if(specimenArm.getCurrent(CurrentUnit.AMPS)>3.5){
                        specimenArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        specimenArmHasPid = true;
                        specimenArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }else{
                        specimenArm.setPower(-0.4);
                    }

                }

                if (gamepad2.dpad_right && !dPadRightPressed) {

                    if (SpecimenMode) {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                        specArmPos=var.speciArmScore;
                        runningActions.add(

                                        specigrabber.SpeciRotateScore()

                        );
                        specigrabberIsOpen = true;
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    }
                }
                dPadRightPressed = gamepad2.dpad_right;



                if (gamepad2.dpad_down && vSlidesCurrentPos > 50 && !dPadDownPressed && !outtakeIsOut && BasketMode) {
                    vSlidesPos = 0;


                } else if(gamepad2.dpad_down && vSlidesCurrentPos > 50 && !dPadDownPressed && outtakeIsOut && BasketMode){
                    runningActions.add(
                            outtakeLM2.OuttakeIdle()
                    );
                    outtakeIsOut=false;

                }else if(gamepad1.dpad_down && !dPadDownPressed && BasketMode){
                    runningActions.add(
                            handLM3.HandStop()
                    );
                    if(hSlidesCurrentPos > 180) {

                        frontIntakeActive = true;
                        currentlyIntaking = true;
                        ejectStage = false;
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_GRAY);

                            runningActions.add(
                                    new ParallelAction(
                                            wrist.WristToDist(var.FrontIntakeWristPos),
                                            elbow.ElbowToDist(var.FrontIntakeElbowPos),
                                            handLM3.HandIntake()
                                    )
                            );

                    }

                } else if(gamepad2.dpad_down && !dPadDownPressed && SpecimenMode){
                    specArmPos = specimenGrabOffset;
                    runningActions.add(
                            new ParallelAction(
                                    specigrabber.SpeciRotateGrab()
                            )
                    );
                    specigrabberIsOpen = true;

                }
                dPadDownPressed = gamepad1.dpad_down || gamepad2.dpad_down;



                if (gamepad2.left_stick_button&&!debugModeIsOn&&!gamepad2.right_stick_button){

                    BasketMode = false;
                    SpecimenMode = true;
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    runningActions.add(
                            outtakeLM2.OuttakeIdle()
                    );
                    outtakeIsOut = false;
                }

                if (gamepad2.right_stick_button&&!debugModeIsOn&&!gamepad2.left_stick_button){

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
                if(hSlidesCurrentPos>250|| lArmCurrentPos>1400|| rArmCurrentPos>1400||currentlyIntaking){
                    driveSpeed = 2.2;
                } else{
                    driveSpeed = 1.2;
                }



                //Failsafes
                if(vSlidesCurrentPos<400&& outtakeIsOut && !specigrabberIsOpen){
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
                telemetry.addData("specCURRENT = ", specimenArm.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("specHasPid = ", specimenArmHasPid);
                telemetry.addData("currentRightArmPos = ", rArmCurrentPos);
                telemetry.addData("currentLeftArmPos = ", lArmCurrentPos);
                telemetry.addData("HSlideTransferVar: = ", hSlideTransferCheck);
                telemetry.addData("FrontIntakeVar: = ", frontIntakeActive);
                telemetry.addData("SpecArmPosVar: = ", specArmPos);
                telemetry.addData("SpecArmPos: = ", specimenArm.getCurrentPosition());
                telemetry.addData("SpecArmPower: = ", (ffSPEC+pidSPEC));
                telemetry.addData("VSlidesPosVariable = ", vSlidesPos);
                telemetry.addData("currentHorizontalSlidesPos = ", hSlidesCurrentPos);
                telemetry.addData("HSlidesPosVariable = ", hSlidesPos);
                telemetry.addData("BasketMode", BasketMode);
                telemetry.addData("Currently Intaking: = ", currentlyIntaking);
                telemetry.addData("Transfer: = ", SampleTransferred);
                telemetry.addData("DebugModeSet: = ", debugModeSet);
                telemetry.addData("DebugMode: = ", debugModeIsOn);
                telemetry.addData("YELLOW DETECTED", pin0.getState()&&pin1.getState());
                telemetry.addData("BLUE DETECTED", pin0.getState()&&!pin1.getState());
                telemetry.addData("RED DETECTED", pin1.getState()&&!pin0.getState());
                telemetry.update();

                dash.sendTelemetryPacket(packet);

            }
        }
    }

}
