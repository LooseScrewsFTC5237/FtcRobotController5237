package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BeamBrakeSubsystem {
    private DigitalChannel rightBeam;
    private DigitalChannel leftBeam;

// robot you better work or else
    private Telemetry telemetry;

    private double count = 0;
    private boolean ballDetectedRight = false;
    private boolean ballDetectedLeft = false;

    public BeamBrakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        rightBeam = hardwareMap.get(DigitalChannel.class, "rightBeamBreak");
        leftBeam = hardwareMap.get(DigitalChannel.class, "leftBeamBreak");
        rightBeam.setMode(DigitalChannel.Mode.INPUT);
        leftBeam.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean Status(){
        if(rightBeam.getState() & !ballDetectedRight){
            count++;
            ballDetectedRight = true;
        }

        if(leftBeam.getState() & !ballDetectedLeft){
            count++;
            ballDetectedLeft = true;
        }

        if(!rightBeam.getState()){
            ballDetectedRight = false;
        }

        if(!leftBeam.getState()){
            ballDetectedLeft = false;
        }

        telemetry.addData("Count: ", count);

        if(count >= 4){
            return true;
        } else {
            return false;
        }

    }

    public void Reset(){
        count = 0;
    }
}
