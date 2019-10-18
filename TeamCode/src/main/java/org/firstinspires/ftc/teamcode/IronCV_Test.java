package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;

@TeleOp(name="CV Test")
public class IronCV_Test extends LinearOpMode {

    private OpenCvCamera phoneCam;
    private OpenCvWebcam webcam;
    private SkystoneDetector skystoneDetector;

    @Override
    public void runOpMode(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();

        skystoneDetector = new SkystoneDetector();
        webcam.setPipeline(skystoneDetector);

        webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()){
            
            telemetry.addData("Stone Position X", skystoneDetector.getScreenPosition().x);
            telemetry.addData("Stone Position Y", skystoneDetector.getScreenPosition().y);
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

            if(gamepad1.a){
                webcam.stopStreaming();
            }else if(gamepad1.x){
                webcam.pauseViewport();
            }else if(gamepad1.y){ webcam.resumeViewport(); }

        }
    }

}
