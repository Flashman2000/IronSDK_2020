package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveOp_BETA extends LinearOpMode {

    Gamepad gp1;

    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;

    DcMotor leftColl = null;
    DcMotor rightColl = null;
    DcMotor spool = null;


    @Override
    public void runOpMode(){

        frontLeft = hardwareMap.get(DcMotor.class, "LF");
        frontRight = hardwareMap.get(DcMotor.class, "RF");
        backLeft = hardwareMap.get(DcMotor.class, "LB");
        backRight = hardwareMap.get(DcMotor.class, "RB");

        leftColl = hardwareMap.get(DcMotor.class, "leftcoll");
        rightColl = hardwareMap.get(DcMotor.class, "rightcoll");
        spool = hardwareMap.get(DcMotor.class, "spool");


        waitForStart();

        while (opModeIsActive()){

            double r = Math.hypot(gp1.left_stick_x, gp1.left_stick_y);
            double robotAngle = Math.atan2(gp1.left_stick_y, gp1.left_stick_x) - Math.PI / 4;
            double rightX = gp1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            double leftCollPwr = gp1.right_trigger;
            double rightCollPwr = -gp1.right_trigger;

            //leftFront.setPower(v1);
            //rightFront.setPower(v2);
            //leftRear.setPower(v3);
            //rightRear.setPower(v4);
            frontLeft.setPower(v1);
            frontRight.setPower(v2);
            backLeft.setPower(v3);
            backRight.setPower(v4);

            leftColl.setPower(leftCollPwr);
            rightColl.setPower(rightCollPwr);

            if(gp1.dpad_up){
                spool.setPower(1);
            }else{ spool.setPower(0); }

            if(gp1.dpad_down){
                spool.setPower(-1);
            }else{ spool.setPower(0); }

        }

    }
}
