package org.firstinspires.ftc.teamcode.Into_The_Deep_Code.LEAGUE_MEET_3.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.roadrunnerStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Elbow;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Hand;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.HorizontalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Lights;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Specigrabber;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.Wrist;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM2_SUBSYSTEMS.OuttakeLM2;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;

@Autonomous(name = "LM3 - Basket 4 Samples", group = "Autonomous")
public class LM3_Basket_4_Samples extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(36, 64.25, Math.toRadians(180));

        // INSTANTIATE SUBSYSTEMS AND DT

        // I made all the subsystem classes check them out to understand how custom actions work!!!!

        SubsystemsVariables var = new SubsystemsVariables();

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        HorizontalSlides hslide = new HorizontalSlides(hardwareMap);

        VerticalSlides vslides = new VerticalSlides(hardwareMap);

        Elbow elbow = new Elbow(hardwareMap);

        Wrist wrist = new Wrist(hardwareMap);

        Hand hand = new Hand(hardwareMap);

        Lights lights = new Lights(hardwareMap);

        OuttakeLM2 outtakeLM2 = new OuttakeLM2(hardwareMap);

        Specigrabber specigrabber = new Specigrabber(hardwareMap);

        Actions.runBlocking(

                // MAKE SERVOS STAY IN PLACE AND STUFF

                new ParallelAction(
                        outtakeLM2.OuttakeIdle(),
                        specigrabber.SpecigrabberOpen(),
                        elbow.ElbowMiddle(),
                        wrist.WristMiddle(),
                        lights.LightsBlue(),
                        hand.HandStop(),
                        hslide.HSlideTo0()
                )
        );

        waitForStart();

        int hSlideGrabExtension = var.hSlideRuleMax;

        int VSlideTempVelo = 1300;

        double sleepTime = 1;

        TrajectoryActionBuilder path = drive.actionBuilder(beginPose)
                //trajectories go backwards
                .setReversed(true)

                //move slides to high basket pos
                .afterTime(0, vslides.SetPosition(2377))

                .strafeToLinearHeading(new Vector2d(36, 61), Math.toRadians(180))

                //grabber out of way
                .afterTime(0, specigrabber.SpecigrabberClose())

                //robot prepares the outtake to score faster
                .afterTime(1.5, outtakeLM2.OuttakeOut())

                //prepare to intake sample TWO while scoring sample ONE
                .afterTime(1, hslide.HSlideToDist(hSlideGrabExtension))
                .afterTime(1, wrist.WristIntake())
                .afterTime(2, elbow.PrepElbowIntake())
                .afterTime(2, hand.HandIntake())

                /*
                This strafeTo moves to the basket, afterTime() actions happen after whatever trajectory
                comes next so all the actions before this happen "dt" seconds after this movement,
                 */
                .strafeToLinearHeading(new Vector2d(59, 52.5), Math.toRadians(255.5))

                .waitSeconds(sleepTime)

                //score sample number ONE

                //grab sample TWO and bring back the outtake
                .afterTime(0, elbow.ElbowIntake())
                .afterTime(0, outtakeLM2.OuttakeIdle())
                .waitSeconds(sleepTime)

                //lower slides back to successfully transfer the next sample
                .afterTime(0,vslides.SetPosition(0))
                .waitSeconds(sleepTime)

                //prepare to transfer sample TWO to the outtake
                .afterTime(0, hand.HandStop())
                .afterTime(0, wrist.WristTransfer())
                .afterTime(0, elbow.ElbowTransfer())
                .afterTime(0, hslide.HSlideToTransfer())
                .waitSeconds(0.75)

                //transfer sample TWO

                //this is only to be safe, if the wrist and elbow can
                //be in transfer position while the slides come down
                //just do that instead
                .afterTime(0.5, hand.HandOuttake())
                .waitSeconds(sleepTime)

                //prepare to intake sample THREE while scoring sample TWO
                .afterTime(0, hand.HandStop())
                .afterTime(0, hslide.HSlideToDist(hSlideGrabExtension-45))
                .afterTime(0, wrist.WristIntake())
                .afterTime(0, elbow.ElbowMiddle())
                .afterTime(0, vslides.SetPosition(2377))
                .waitSeconds(1.25)

                //score sample TWO
                .afterTime(0, outtakeLM2.OuttakeOut())
                .waitSeconds(sleepTime)

                //reset outtake after scoring sample 2
                .afterTime(0, outtakeLM2.OuttakeIdle())

                //move to next scoring position
                .turn(Math.toRadians(14.5))
                .waitSeconds(0.75)

                //lower slides to transfer sample 3
                .afterTime(0, vslides.SetPosition(0))
                .waitSeconds(0.5)

                //prepare to grab sample 3
                .afterTime(0, elbow.PrepElbowIntake())
                .afterTime(0, hand.HandIntake())
                .waitSeconds(sleepTime)

                //grab sample 3
                .afterTime(0, elbow.ElbowIntake())
                .waitSeconds(sleepTime)

                //prepare for transfer
                .afterTime(0, hand.HandStop())
                .afterTime(0, wrist.WristTransfer())
                .afterTime(0, elbow.ElbowTransfer())
                .afterTime(0, hslide.HSlideToTransfer())
                .waitSeconds(0.75)

                //transfer sample 3
                .afterTime(0.5, hand.HandOuttake())
                .waitSeconds(1)



                //prepare to intake sample FOUR while scoring sample THREE
                .afterTime(0, hand.HandStop())
                .afterTime(0, wrist.WristIntake())
                .afterTime(0, elbow.ElbowMiddle())
                .afterTime(0, hslide.HSlideToDist(hSlideGrabExtension-5))
                .afterTime(0, vslides.SetPosition(2377))
                .turn(Math.toRadians(-14.5))
                .waitSeconds(1)

                //score sample 3
                .afterTime(0, outtakeLM2.OuttakeOut())
                .waitSeconds(sleepTime)

                //reset outtake after scoring sample 3
                .afterTime(0, outtakeLM2.OuttakeIdle())

                //move to next scoring position
                .turn(Math.toRadians(40))
                .waitSeconds(0.75)

                //lower slides to transfer sample 4
                .afterTime(0, vslides.SetPosition(0))
                .waitSeconds(0.5)

                //prepare to grab sample 4
                .afterTime(0, elbow.PrepElbowIntake())
                .afterTime(0, hand.HandIntake())
                .waitSeconds(sleepTime)

                //grab sample 4
                .afterTime(0, elbow.ElbowIntake())
                .waitSeconds(sleepTime)

                //prepare for transfer
                .afterTime(0, hand.HandStop())
                .afterTime(0, wrist.WristTransfer())
                .afterTime(0, elbow.ElbowTransfer())
                .afterTime(0, hslide.HSlideToTransfer())
                .waitSeconds(0.75)

                //transfer sample 4
                .afterTime(0.5, hand.HandOuttake())
                .waitSeconds(1)



                //prepare to score sample 4
                .afterTime(0, hand.HandStop())
                .afterTime(0, wrist.WristIntake())
                .afterTime(0, elbow.ElbowMiddle())
                .afterTime(0, vslides.SetPosition(2377))
                .turn(Math.toRadians(-40))
                .waitSeconds(1)

                //score sample 4
                .afterTime(0, outtakeLM2.OuttakeOut())
                .waitSeconds(sleepTime)

                //reset outtake and lower slides
                .afterTime(0, outtakeLM2.OuttakeIdle())
                .waitSeconds(sleepTime)
                .afterTime(0, vslides.SetPosition(735))
                //park in the designated area
                .strafeToLinearHeading(new Vector2d(40,12), Math.toRadians(0))
                .strafeTo(new Vector2d(21,12));


        if (isStopRequested()) { return; }

        Actions.runBlocking(
                new ParallelAction(
                        vslides.UpdatePID(),
                        path.build()
                )
        );
    }
}
