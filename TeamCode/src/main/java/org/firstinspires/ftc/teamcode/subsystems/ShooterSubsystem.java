package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem {

    private double shooterSpeed = 1000;
    private DcMotorEx shooter;

    private Telemetry telemetry;

    public ShooterSubsystem(HardwareMap hardwareMap,Telemetry telemetry){
        this.telemetry = telemetry;
        shooter = hardwareMap.get(DcMotorEx.class,"Shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runShooter(){
       shooter.setVelocity(shooterSpeed);
    }

    public void stopShooter(){
        shooter.setVelocity(0);
    }
}
