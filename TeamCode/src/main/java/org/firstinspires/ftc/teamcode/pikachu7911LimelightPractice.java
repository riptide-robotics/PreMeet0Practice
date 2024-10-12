package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class pikachu7911LimelightPractice extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limecam");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0); // What is pipelineSwitch?

        //int[] validIDs = {3,4};
        //LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);

        /*
         * Starts polling for data
         */
        limelight.start();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                }
            }
        }
    }
}
