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

    String intakeCurrentMode = "Idle";
    String collCurrentOrientation = "Stowed";

    @Override
    public void runOpMode() {

        leftColl = hardwareMap.get(DcMotor.class, "leftcoll");
        rightColl = hardwareMap.get(DcMotor.class, "rightcoll");

        leftPiv = hardwareMap.get(Servo.class, "leftpiv");
        rightPiv = hardwareMap.get(Servo.class, "rightpiv");

        rightColl.setDirection(DcMotorSimple.Direction.REVERSE);
        rightColl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftColl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightPiv.setPosition(0);
        leftPiv.setPosition(1);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) {

                leftColl.setPower(0);
                rightColl.setPower(0);
                intakeCurrentMode = "Idle";

            }

            if (gamepad1.right_bumper) {
                leftColl.setPower(0.5);
                rightColl.setPower(0.5);
                intakeCurrentMode = "Collecting";

            }

            if (gamepad1.left_bumper) {
                leftColl.setPower(-0.7);
                rightColl.setPower(-0.7);
                intakeCurrentMode = "Ejecting";
            }

            if (gamepad1.dpad_up) {

                rightPiv.setPosition(1);
                leftPiv.setPosition(0);
                collCurrentOrientation = "Deployed";

            }

            if (gamepad1.dpad_down) {

                rightPiv.setPosition(0);
                leftPiv.setPosition(1);
                collCurrentOrientation = "Stowed";

            }

            telemetry.addData("Collection Status: ", intakeCurrentMode);
            telemetry.addData("Current Collection Orientation: ", collCurrentOrientation);
            telemetry.update();

        }
    }
}
