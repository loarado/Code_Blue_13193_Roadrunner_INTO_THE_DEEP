package org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM2_SUBSYSTEMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.tuning.variables_and_subsystemClasses.LM1_SUBSYSTEMS.VerticalSlides;

public class CancelableVSlidesAction implements Action {
    private final Action action; // Use the generic Action type
    private final VerticalSlides slidesInstance;
    private boolean cancelled = false;

    public CancelableVSlidesAction(VerticalSlides slidesInstance, int targetPosition) {
        this.slidesInstance = slidesInstance;
        this.action = slidesInstance.VSlidesToDistPIDF(targetPosition); // Assign the Action
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (cancelled) {
            slidesInstance.stopSlides(); // Stop slides when canceled
            return false; // Action is no longer running
        }

        return action.run(telemetryPacket); // Delegate to the inner action
    }

    public void cancelAbruptly() {
        cancelled = true;
    }
}
