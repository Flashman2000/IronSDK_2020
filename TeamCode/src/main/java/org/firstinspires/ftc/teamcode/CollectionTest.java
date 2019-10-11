package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Collection Test")
public class CollectionTest extends LinearOpMode {

    DcMotor leftColl = null;
    DcMotor rightColl = null;

    Servo leftPiv = null;
    Servo rightPiv = null;

    @Override
    public void runOpMode(){

        leftColl = hardwareMap.get(DcMotor.class, "leftcoll");
        rightColl = hardwareMap.get(DcMotor.class, "rightcoll");

        leftPiv = hardwareMap.get(Servo.class, "leftpiv");
        rightPiv = hardwareMap.get(Servo.class, "rightpiv");

        rightColl.setDirection(DcMotorSimple.Direction.REVERSE);
        rightColl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftColl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            double leftCollPwr = gamepad1.right_trigger * 0.5;
            double rightCollPwr = gamepad1.right_trigger * 0.5;

            leftColl.setPower(leftCollPwr);
            rightColl.setPower(rightCollPwr);

            if(gamepad1.a){
                rightPiv.setPosition(1);
            }
            if(gamepad1.b){
                rightPiv.setPosition(0);
            }
            if(gamepad1.x){
                leftPiv.setPosition(1);
            }
            if(gamepad1.y){
                leftPiv.setPosition(0);
            }


        }

    }

}
