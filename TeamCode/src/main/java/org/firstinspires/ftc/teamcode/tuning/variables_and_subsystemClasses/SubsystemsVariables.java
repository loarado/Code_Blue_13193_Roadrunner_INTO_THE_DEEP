package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses;

public class SubsystemsVariables {

    public SubsystemsVariables() {

        //CONSTRUCTOR

    }


    /* This class just contains important variables related to robot
    subsystems so if we need to change any we can do it easily without
    causing any problems. You welcome :) */



    public final double OuttakeWristPos = 0.62;
    public final double IntakeWristPos = 0.38;
    public final double MiddleWristPos = 0.5;
    public final double FrontIntakeWristPos = 0.55;



    public final double OuttakeElbowPos = 0.38;
    public final double IntakeElbowPos = 0.15;
    public final double PrepIntakeElbowPos = 0.22;
    public final double MiddleElbowPos = 0.4;
    public final double FrontIntakeElbowPos = 0.05;
    public final double EjectElbowPos = 0.3;


    public final double rightHandIn = 1;
    public final double rightHandOut = -1;
    public final double rightHandStop = 0;

    public final double leftHandIn = -1;
    public final double leftHandOut = 1;
    public final double leftHandStop = 0;




    public final int vSlidePhysicalMax = 3700;     // Max encoder tick value for vertical slides
    public final double vSlideMaxHeight = 63.5;    // Full extension height of slides in inches
    public final double bottomOffset = 13;         // Offset for the bottom of the outtake box, in inches
    public final double extraClearance = 2;        // Additional clearance needed for game element deposit

    public final double ticksPerInch = vSlidePhysicalMax / vSlideMaxHeight;

    public final double highBucketHeight = 43;     // Height of high bucket in inches
    public final double lowBucketHeight = 25.75;   // Height of low bucket in inches

    public final int vSlideHighBasket = (int) ((highBucketHeight + bottomOffset + extraClearance) * ticksPerInch);
    public final int vSlideLowBasket = (int) ((lowBucketHeight + bottomOffset + extraClearance) * ticksPerInch);

    public final double clawHeight = 9.5;            // Height of the claw from the ground in inches

    // Chamber heights (measured from ground)
    public final double highChamberHeight = 26;       // Height of high chamber in inches
    public final double lowChamberHeight = 13;        // Height of low chamber in inches
    public final double approachClearance = 1;        // Clearance above the chamber bar for initial clipping
    public final double dropClearance = 1.5;            // Drop below the chamber bar for releasing the element

    // Encoder tick values for high and low chambers, positioning above the bars for clipping
    public final int vSlideHighChamber = (int) ((highChamberHeight - clawHeight + approachClearance) * ticksPerInch);
    public final int vSlideLowChamber = (int) ((lowChamberHeight - clawHeight + approachClearance) * ticksPerInch);

    // Encoder tick values for dropping the element below the bars
    public final int vSlideHighChamberDrop = (int) ((highChamberHeight - clawHeight - dropClearance) * ticksPerInch);
    public final int vSlideLowChamberDrop = (int) ((lowChamberHeight - clawHeight - dropClearance) * ticksPerInch);

    public final double vSlideSpeed = 0.5;
    public final double vSlideVelocity = 1300;




    public final double robotLength = 15.5;
    public final double backExtension = 5;
    public final double slidesLength = 23.25;

    public final int hSlidePhysicalMax = 620;
    public final double hSlideSpeed = 0.5;
    public final double hSlideVelocity = 1300;
    public final int hSlideOuttakePos = 50;

    public final int hSlideRuleMax = (int) (hSlidePhysicalMax*((slidesLength-((slidesLength + robotLength + backExtension) - 42))/slidesLength));



    public final double outtakeIdle = 1;
    public final double outtakeOut = 0.33;



    public final double specigrabberOpen = 0.55;
    public final double specigrabberClosed = 0.82;


    final double servoTolerance = 0.03;

}