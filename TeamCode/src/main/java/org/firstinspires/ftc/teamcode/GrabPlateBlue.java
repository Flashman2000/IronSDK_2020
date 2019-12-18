package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous
public class GrabPlateBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);

        drive.primeBack();
        drive.releaseBack();
        drive.releaseFront();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .back(22)
                .strafeRight(17)
                .back(4)
                .build()
        );

        drive.clampBack();
        sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .forward(15)
                .build()
        );

        drive.turnLeftGyro(Math.toRadians(90), this, telemetry);
        drive.unclampBack();
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .forward(40)
                .build()
        );
        drive.leftArm.setPosition(1);
        drive.rightArm.setPosition(0);
        sleep(2000);

    }

}
