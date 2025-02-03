package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LT_SUBSYSTEMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.VARIABLES.SubsystemsVariables;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;

public class DistLT {

    //Instantiate the sensor/servo/motor
    public DistanceSensor distLeft;

    // Import final variables
    SubsystemsVariables var = new SubsystemsVariables();


    public DistLT(HardwareMap hardwareMap) {

        //Constructor
        distLeft = (hardwareMap.get(DistanceSensor.class, "distLeft"));

    }

    // Actions are below
    
    public class distGreater implements Action  {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            return distLeft.getDistance(DistanceUnit.INCH)<6;
        }
    }
    public Action DistGreater() {
        return new distGreater();
    }



    }