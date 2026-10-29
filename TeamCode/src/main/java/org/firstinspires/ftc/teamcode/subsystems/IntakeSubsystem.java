package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {
    private DcMotor leftIntakeMotor;

    private DcMotor rightIntakeMotor;

    private Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        leftIntakeMotor = hardwareMap.get(DcMotor.class, "IntakeLEFT");
        rightIntakeMotor = hardwareMap.get(DcMotor.class, "IntakeRIGHT");

        leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void runIntakeForward(){
        leftIntakeMotor.setPower(1);
        rightIntakeMotor.setPower(1);
    }

    public void runIntakeReverse(){
        leftIntakeMotor.setPower(-1);
        rightIntakeMotor.setPower(-1);
    }

    public void stopIntake(){
        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);
    }
}
