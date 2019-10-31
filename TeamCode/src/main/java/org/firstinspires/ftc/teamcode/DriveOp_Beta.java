package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.opencv.core.Mat;

@TeleOp(name = "DRIVE")
public class DriveOp_Beta extends LinearOpMode {

    @Override
    public void runOpMode(){

        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);



        waitForStart();

        while(opModeIsActive()){

            double r1 = gamepad1.left_stick_x;
            double r2 = -gamepad1.left_stick_y;
            double t = gamepad1.right_stick_x;

            double rfpwr = -r1 + r2 + t;
            double rbpwr =  r1 + r2 - t;
            double lfpwr =  r1 + r2 + t;
            double lbpwr = -r1 + r2 - t;

            if(gamepad1.a){

                drive.setBackServoPos(1, 1);

            }

            if(gamepad1.b){

                drive.setBackServoPos(0, 0);

            }

            if(gamepad1.dpad_up){
                drive.setFrontArmServoPos(1, 1);
            }
            if(gamepad1.dpad_down){
                drive.setFrontArmServoPos(0, 0);
            }

            drive.setMotorPowers(lfpwr, lbpwr, rbpwr, rfpwr);


        }

    }

}
