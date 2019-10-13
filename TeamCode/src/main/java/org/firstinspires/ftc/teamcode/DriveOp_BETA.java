package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveOp_BETA extends LinearOpMode {

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

            /**
             * Various equations to set up motor powers before they are staged
             * to the motor
             */

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            double leftCollPwr = gamepad1.right_trigger;
            double rightCollPwr = -gamepad1.right_trigger;

            /**
             * staging the created power values
             */

            frontLeft.setPower(v1);
            frontRight.setPower(v2);
            backLeft.setPower(v3);
            backRight.setPower(v4);

            leftColl.setPower(leftCollPwr);
            rightColl.setPower(rightCollPwr);

            /**
             * boolean loops for motor/servo control
             */

            if(gamepad1.dpad_up){
                spool.setPower(1);
            }else{ spool.setPower(0); }

            if(gamepad1.dpad_down){
                spool.setPower(-1);
            }else{ spool.setPower(0); }

        }

    }
}
