package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {
    private DcMotorEx leftIntakeMotor;

    private DcMotorEx rightIntakeMotor;

    private Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        leftIntakeMotor = hardwareMap.get(DcMotorEx.class, "leftIntakeMotor");
        rightIntakeMotor = hardwareMap.get(DcMotorEx.class, "rightIntakeMotor");
    }


    public void runIntakeForward(){

    }

    public void runIntakeReverse(){

    }
}
