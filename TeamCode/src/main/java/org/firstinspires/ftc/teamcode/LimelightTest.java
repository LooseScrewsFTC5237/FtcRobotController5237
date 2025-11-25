package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class LimelightTest extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();

    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        Pose3D botpose = result.getBotpose();

        telemetry.addData("tx", result.getTx());
        telemetry.addData("ty", result.getTy());
        telemetry.addData("Botpose", botpose.toString());
    }

}
