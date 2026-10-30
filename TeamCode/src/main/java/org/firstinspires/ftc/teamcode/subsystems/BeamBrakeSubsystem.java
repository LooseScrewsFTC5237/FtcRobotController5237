package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BeamBrakeSubsystem {
    private DigitalChannel rightBeam;
    private DigitalChannel leftBeam;

// robot you better work or else
    private Telemetry telemetry;

    public BeamBrakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        rightBeam = hardwareMap.get(DigitalChannel.class, "rightBeamBreak");
        leftBeam = hardwareMap.get(DigitalChannel.class, "leftBeamBreak");

    }

    public void Status(){
         
    }
}
