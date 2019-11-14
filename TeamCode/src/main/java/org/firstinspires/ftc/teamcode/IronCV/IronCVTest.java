package org.firstinspires.ftc.teamcode.IronCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="IronCV Test")
public class IronCVTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private String pos;

    IronCVDetectorClass detector = new IronCVDetectorClass(1f/8f, 3f/8f);

    int[] vals;

    @Override
    public void runOpMode() throws InterruptedException{

        detector.camSetup(hardwareMap);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()){

            detector.updateVals();
            vals = detector.getVals();

            if(vals[1] < 200 && vals[0] > 210 && vals[2] > 210){
                pos = "Left";
            }else if(vals[1] > 210 && vals[0] < 200 && vals[2] > 210){
                pos = "Center";
            }else if(vals[1] > 210 && vals[0] > 210 && vals[2] < 200){
                pos = "Right";
            }else{
                pos = "NOT FOUND";
            }

            telemetry.addData("Position", pos);
            telemetry.update();
            sleep(100);

        }

    }
}
