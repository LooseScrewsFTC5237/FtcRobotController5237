package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ShooterSubsystem {

    public static double shooterSpeed = 1500;

    private DcMotorEx shooter;

    private Telemetry telemetry;

    public ShooterSubsystem(HardwareMap hardwareMap,Telemetry telemetry){
        this.telemetry = telemetry;
        shooter = hardwareMap.get(DcMotorEx.class,"Shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initShooter(){
        PIDFCoefficients pidfNew = new PIDFCoefficients (200, 0, 0, 22.16);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

    }

    public void runShooter(){
       shooter.setVelocity(shooterSpeed);
    }

    public void stopShooter(){
        shooter.setVelocity(0);
    }
}
