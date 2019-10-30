package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.opencv.core.Mat;

@TeleOp(name = "DRIVE")
public class DriveOp_Beta extends LinearOpMode {

    DcMotor frontLeft  = null;
    DcMotor frontRight = null;
    DcMotor backLeft   = null;
    DcMotor backRight  = null;

    @Override
    public void runOpMode(){

        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            double r           = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle  = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX      = gamepad1.right_stick_x;

            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            drive.setMotorPowers(v1, v3, v4, v2);

        }

    }

}
