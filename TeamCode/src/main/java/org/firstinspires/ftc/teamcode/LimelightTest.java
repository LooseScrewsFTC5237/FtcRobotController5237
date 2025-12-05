package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class LimelightTest extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        imu = hardwareMap.get(IMU.class,  "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));


        limelight.start();
    }

    @Override
    public void loop() {

      //  YawPitchRollAngles orientation =imu.getRobotYawPitchRollAngles();
        //limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            telemetry.addData("Target x", llResult.getTx());
            telemetry.addData("Target y", llResult.getTy());
            telemetry.addData("Target area", llResult.getTa());
            LLResultTypes.FiducialResult goalTag = null;
            for (LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                telemetry.addLine(String.format("Found tag: %d - %s", fiducial.getFiducialId(), fiducial));
                if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24) {
                    telemetry.addLine(String.format("Found a goal tag! %d", fiducial.getFiducialId()));
                    goalTag = fiducial;
                }
            }
            if (goalTag != null) {
                Position p = goalTag.getTargetPoseCameraSpace().getPosition();
                double distance = Math.hypot(p.x, p.z);
                telemetry.addData("distance", (Math.hypot(p.x, p.z) * 39.3701));
                telemetry.addData("x", p.x);
                telemetry.addData("z", p.z);
            }


        }
    }



}
