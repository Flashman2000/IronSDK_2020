package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.math.Line;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="IronCV Test")
public class IronCVTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

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
            telemetry.addData("Values", vals[1]+"   "+vals[0]+"   "+vals[2]);
            telemetry.update();
            sleep(100);

        }

    }
}
