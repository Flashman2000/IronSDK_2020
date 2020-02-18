package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.MecanumPowers;
import org.firstinspires.ftc.teamcode.util.MecanumUtil;

@Autonomous
public class RR_Test extends LinearOpMode {
    String SkyStoneLocation;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap, true);

        drive.primeBack();
        drive.releaseFront();
        drive.releaseBack();
        drive.setPoseEstimate(new Pose2d(-36, 65, 0));
        drive.primeServo();
        waitForStart();

        SkyStoneLocation = drive.detectSkystone();

        //drive.set

        //drive.alignSkystone(SkyStoneLocation);
        if(SkyStoneLocation == "Left") {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(-32, 35), new ConstantInterpolator(0))
                            .build()
            );
            drive.grabFront();
            sleep(500);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .strafeLeft(4.5)
                    .build()
            );

            while(drive.imu.getAngularOrientation().firstAngle < Math.toRadians(85) && opModeIsActive()) {
                double leftX = -0.1;
                double leftY = 0.7;
                double angle = -Math.atan2(leftY, leftX) + Math.PI / 2;
                angle -= drive.getExternalHeading();
                double driveScale = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
                driveScale = Range.clip(driveScale, 0, 1);
                double turn = Math.copySign(Math.pow(0.4, 2), 0.4);

                MecanumPowers powers = MecanumUtil.powersFromAngle(angle, driveScale, turn);
                drive.setPowers(powers);
                telemetry.addData("HEADING", drive.imu.getAngularOrientation().firstAngle);
                telemetry.update();

            }
            drive.releaseFront();
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(10)
                            .back(6)
                            .build()
            );

        }
    }
}
