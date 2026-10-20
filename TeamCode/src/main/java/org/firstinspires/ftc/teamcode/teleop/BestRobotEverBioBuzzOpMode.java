package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name = "BioBuzz", group = "Competition")
public class BestRobotEverBioBuzzOpMode extends OpMode {

    private IntakeSubsystem intakeSubsystem;


    @Override
    public void init() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        double driveSpeed, strafe, turn;
        driveSpeed = -gamepad1.left_stick_y * DRIVE_SPEED;
        strafe = gamepad1.left_stick_x  * DRIVE_SPEED;
    }
}
