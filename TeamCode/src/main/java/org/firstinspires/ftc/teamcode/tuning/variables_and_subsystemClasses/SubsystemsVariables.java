package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubsystemsVariables {
    public final double OuttakeWristPos;

    public final double IntakeWristPos;

    public final double MiddleWristPos;


    public final double OuttakeElbowPos;

    public final double IntakeElbowPos;

    public final double MiddleElbowPos;

    public final double rightHandIn;
    public final double rightHandOut;
    public final double rightHandStop;

    public final double leftHandIn;
    public final double leftHandOut;
    public final double leftHandStop;

    public final int hSlidePhysicalMax = 800;
    public final double hSlideSpeed;

    public final int vSlidePhysicalMax;
    public final double vSlideSpeed;

    public final double robotLength = 15.5;
    public final double backExtention = 6;
    public final double slidesLength = 29.25;

    public final int hSlideRuleMax = (int) (hSlidePhysicalMax*((slidesLength-((slidesLength + robotLength + backExtention) - (42)))/slidesLength));

    public SubsystemsVariables() {
        OuttakeWristPos = 0.58;
        IntakeWristPos = 0.38;
        MiddleWristPos = 0.38;

        OuttakeElbowPos = 0.3;
        IntakeElbowPos = 0.43;
        MiddleElbowPos = 0.33;

        rightHandIn = -1;
        rightHandOut = 1;
        rightHandStop = -1;

        leftHandIn = 1;
        leftHandOut = -1;
        leftHandStop = 0;

        hSlideSpeed = 0.5;
        vSlidePhysicalMax = 3800;
        vSlideSpeed = 0.5;
    }
}