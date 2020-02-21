package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubMotor;

@TeleOp
public class odoTracker extends LinearOpMode {


    public ExpansionHubMotor leftEncoder, rightEncoder, backEncoder;


    public void runOpMode() {
        leftEncoder = hardwareMap.get(ExpansionHubMotor.class, "rc");
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightEncoder = hardwareMap.get(ExpansionHubMotor.class, "spool");
        backEncoder = hardwareMap.get(ExpansionHubMotor.class, "spool2");

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("right", rightEncoder.getCurrentPosition());
            telemetry.addData("left", leftEncoder.getCurrentPosition());
            telemetry.addData("back", backEncoder.getCurrentPosition());
            telemetry.update();

            if (gamepad1.a) {
                leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(200);

                leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            }
        }
    }




}
