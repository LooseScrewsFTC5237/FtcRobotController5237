package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "BioBuzz", group = "Competition")
public class BestRobotEverBioBuzzOpMode extends OpMode {

    private IntakeSubsystem intakeSubsystem;
    private MecanumDrive drive;

    @Override
    public void init() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.updatePoseEstimate();

        double y = -gamepad1.left_stick_y; // Forward/Backward
        double x = gamepad1.left_stick_x;  // Strafing left/right
        double rx = gamepad1.right_stick_x;// Rotation

        // Fixed: access pose through the localizer
        Pose2d poseEstimate = drive.localizer.getPose();
        Vector2d input = Rotation2d.exp(-poseEstimate.heading.toDouble()).times(new Vector2d(x, y));
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(input.x, input.y), rx));


        if (gamepad2.right_trigger > 0) {
            intakeSubsystem.runIntakeForward();
        } else if (gamepad2.left_trigger > 0) {
        }
    }
}