package org.firstinspires.ftc.teamcode.auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.CameraTrackingSubsystems;

public class WaitForAprilTagAction implements Action {
    private final CameraTrackingSubsystems camera;
    private final Alliance alliance;

    public WaitForAprilTagAction(CameraTrackingSubsystems camera, Alliance alliance) {
        this.camera = camera;
        this.alliance = alliance;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        return !camera.seeApirltag(alliance); // false stops the race when found
    }
}

