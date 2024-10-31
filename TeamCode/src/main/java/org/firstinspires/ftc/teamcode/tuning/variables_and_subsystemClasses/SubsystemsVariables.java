package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses;

public class SubsystemsVariables {

    public SubsystemsVariables() {

        //CONSTRUCTOR

    }

    /* This class just contains important variables related to robot
    subsystems so if we need to change any we can do it easily without
    causing any problems. You welcome :) */

    public final double OuttakeWristPos = 0.38;

    public final double IntakeWristPos = 0.58;

    public final double MiddleWristPos = 0.5;

    public final double FrontIntakeWristPos = 0.64;


    public final double OuttakeElbowPos = 0.23;

    public final double IntakeElbowPos = 0.55;

    public final double PrepIntakeElbowPos = 0.3;

    public final double MiddleElbowPos = 0.5;

    public final double FrontIntakeElbowPos = 0.1;

    public final double rightHandIn = -1;
    public final double rightHandOut = 1;
    public final double rightHandStop = 0;

    public final double leftHandIn = 1;
    public final double leftHandOut = -1;
    public final double leftHandStop = 0;

    public final int hSlidePhysicalMax = 800;
    public final double hSlideSpeed = 0.5;
    public final double hSlideVelocity = 1300;

    public final int vSlidePhysicalMax = 3800;
    public final int vSlideHighBasket = 3000;
    public final int vSlideLowBasket = 2000;

    public final int vSlideSpeicmenLow = 1000;
    public final int vSlideSpeicmenLow2 = vSlideSpeicmenLow - 350;
    public final int vSlideSpecimenHigh = 1500;
    public final int vSlideSpecimenHigh2 = vSlideSpecimenHigh - 350;

    public final double vSlideSpeed = 0.5;
    public final double vSlideVelocity = 1300;

    public final double robotLength = 15.5;
    public final double backExtension = 6;
    public final double slidesLength = 29.25;

    public final int hSlideRuleMax = (int) (hSlidePhysicalMax*((slidesLength-((slidesLength + robotLength + backExtension) - (42)))/slidesLength));

    public final double outtakeIdle = 0.4;
    public final double outtakeOut = 0.9;

    public final double specigrabberOpen = 0.55;
    public final double specigrabberClosed = 0.82;

}