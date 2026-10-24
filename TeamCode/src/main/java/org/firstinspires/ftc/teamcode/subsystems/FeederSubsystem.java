package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FeederSubsystem {
    private DcMotor indexer;
    private Telemetry telemetry;

    public FeederSubsystem(HardwareMap hardwareMap,Telemetry telemetry){
        this.telemetry = telemetry;
        indexer = hardwareMap.get(DcMotor.class, "indexer");
    }

    public void runIndexerForward(){
        indexer.setPower(1);
    }

    public void runIndexerReverse(){
        indexer.setPower(-1);
    }

    public void stopIndexer(){
        indexer.setPower(0);
    }
}
