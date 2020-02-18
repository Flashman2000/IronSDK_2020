package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configs.Field_Locations;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.revextensions2.ExpansionHubEx;

import java.text.Format;

@Autonomous
public class new_blue2 extends LinearOpMode {

    String SkystoneLocation;

    //boolean highVoltage = false;

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap, true);

        double voltage = drive.hub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
        double voltage2 = drive.hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);

        Field_Locations locations = new Field_Locations();

        ConstantInterpolator FACING_LZ = new ConstantInterpolator(Math.toRadians(0));

        drive.frontYkA.setPosition(0.75);

        telemetry.addLine("Ready");
        telemetry.addData("Voltage 1", voltage);
        if(voltage < 13.35){
            telemetry.addLine("CRITICAL WARNING:");
            telemetry.addLine("VOLTAGE IS TOO LOW, EXPECT INSTABILITY");
            telemetry.addLine("13.6 IS OPTIMAL VOLTAGE");
        }

        if(voltage > 13.8){
            telemetry.addLine("CRITICAL WARNING:");
            telemetry.addLine("VOLTAGE IS TOO DAMN HIGH, EXPECT INSTABILITY");
            telemetry.addLine("13.6 IS OPTIMAL VOLTAGE");
        }

        telemetry.update();

        waitForStart();

        SkystoneLocation = drive.detectSkystone();

        drive.setPoseEstimate(new Pose2d(-36, 65, Math.toRadians(0)));

        telemetry.addData("Location", SkystoneLocation);
        telemetry.update();

        if(SkystoneLocation == "Left" || SkystoneLocation == "NOT FOUND"){
            //97

            drive.backYkA.setPosition(0.6);
            drive.backYk.setPosition(0.5);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(6)
                            .strafeRight(37.5)
                            .build()
            );

            double offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.setPoseEstimate(locations.blueSkystone4P);

            //TODO: add grab code
            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .strafeLeft(7.5)
                    .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(95)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(11)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.7);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(9)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(65)
                            .build()
            );


            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(5)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(6)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(83)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(8.5)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.backs.setPosition(1);

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.7);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(5)
                            .strafeLeft(5)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.turnTo(90);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(10)
                            .build()
            );

            //TODO: clamp code
            drive.backs.setPosition(0);
            sleep(500);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(20)
                            .build()
            );

            while (drive.imu.getAngularOrientation().firstAngle < Math.toRadians(180) && drive.imu.getAngularOrientation().firstAngle > Math.toRadians(0) && opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                //double pwr = Range.clip(Math.abs(0.011*(drive.imu.getAngularOrientation().firstAngle - Math.toRadians(180))), 0.3, 1);

                if(drive.imu.getAngularOrientation().firstAngle > Math.toRadians(120)){
                    drive.frontYkA.setPosition(0.5);
                }

                drive.LF.setPower(-0.7);
                drive.LB.setPower(0.7);
                drive.RF.setPower(-0.7);
                drive.RB.setPower(0.7);
                telemetry.addData("Heading", drive.imu.getAngularOrientation().firstAngle);
                telemetry.update();
            }
            drive.killAll();


            //TODO: unclamp code

            drive.backs.setPosition(1);

            //TODO: drop front wheels

            drive.leftArm.setPosition(1);
            drive.rightArm.setPosition(0);

            drive.frontYkA.setPosition(0.75);

            drive.setPoseEstimate(new Pose2d(45, -48, Math.toRadians(179)));

            drive.relayPose(telemetry, drive);

            //TODO: set new pose after empirical analysis

            //sleep(500000);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(2)
                            .forward(32)
                            .build()
            );

        }

        if(SkystoneLocation == "Right"){

            drive.backYkA.setPosition(0.6);
            drive.backYk.setPosition(0.5);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .back(22)
                    .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .strafeRight(29)
                    .build()

            );

            drive.setPoseEstimate(locations.blueSkystone6P);
            double offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            //TODO: add grab code
            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(5.5)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(112)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(9.5)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.7);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(9)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(85)
                            .build()
            );


            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(5.5)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(6)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(100)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(8.5)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.backs.setPosition(1);

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.7);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(7)
                            .strafeLeft(5)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.turnTo(90);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(10)
                            .build()
            );

            //TODO: clamp code
            drive.backs.setPosition(0);
            sleep(500);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(18)
                            .build()
            );

            while (drive.imu.getAngularOrientation().firstAngle < Math.toRadians(180) && drive.imu.getAngularOrientation().firstAngle > Math.toRadians(0) && opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                //double pwr = Range.clip(Math.abs(0.011*(drive.imu.getAngularOrientation().firstAngle - Math.toRadians(180))), 0.3, 1);

                if(drive.imu.getAngularOrientation().firstAngle > Math.toRadians(120)){
                    drive.frontYkA.setPosition(0.5);
                }

                drive.LF.setPower(-0.7);
                drive.LB.setPower(0.7);
                drive.RF.setPower(-0.7);
                drive.RB.setPower(0.7);
                telemetry.addData("Heading", drive.imu.getAngularOrientation().firstAngle);
                telemetry.update();
            }
            drive.killAll();


            //TODO: unclamp code

            drive.backs.setPosition(1);

            //TODO: drop front wheels

            drive.leftArm.setPosition(1);
            drive.rightArm.setPosition(0);

            drive.frontYkA.setPosition(0.75);

            drive.setPoseEstimate(new Pose2d(45, -48, Math.toRadians(179)));

            drive.relayPose(telemetry, drive);

            //TODO: set new pose after empirical analysis

            //sleep(500000);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            //.strafeLeft(4)
                            .forward(32)
                            .build()
            );

        }

        if(SkystoneLocation == "Center"){

            drive.backYkA.setPosition(0.6);
            drive.backYk.setPosition(0.5);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(10)
                            .strafeRight(38)
                            .build()
            );

            drive.setPoseEstimate(locations.blueSkystone5P);

            double offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            //TODO: add grab code
            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(5.5)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(105)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(11)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.7);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(9)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(75)
                            .build()
            );


            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(5)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(6)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(100)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(8.5)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.backs.setPosition(1);

            drive.backYkA.setPosition(1);
            sleep(200);
            drive.backYk.setPosition(0.7);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.backYkA.setPosition(0.25);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(5)
                            .strafeLeft(5)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.turnTo(90);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(10)
                            .build()
            );

            //TODO: clamp code
            drive.backs.setPosition(0);
            sleep(500);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(20)
                            .build()
            );

            while (drive.imu.getAngularOrientation().firstAngle < Math.toRadians(180) && drive.imu.getAngularOrientation().firstAngle > Math.toRadians(0) && opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                //double pwr = Range.clip(Math.abs(0.011*(drive.imu.getAngularOrientation().firstAngle - Math.toRadians(180))), 0.3, 1);

                if(drive.imu.getAngularOrientation().firstAngle > Math.toRadians(120)){
                    drive.frontYkA.setPosition(0.5);
                }

                drive.LF.setPower(-0.7);
                drive.LB.setPower(0.7);
                drive.RF.setPower(-0.7);
                drive.RB.setPower(0.7);
                telemetry.addData("Heading", drive.imu.getAngularOrientation().firstAngle);
                telemetry.update();
            }
            drive.killAll();


            //TODO: unclamp code

            drive.backs.setPosition(1);

            //TODO: drop front wheels

            drive.leftArm.setPosition(1);
            drive.rightArm.setPosition(0);

            drive.frontYkA.setPosition(0.75);

            drive.setPoseEstimate(new Pose2d(45, -48, Math.toRadians(179)));

            drive.relayPose(telemetry, drive);

            //TODO: set new pose after empirical analysis

            //sleep(500000);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(2)
                            .forward(32)
                            .build()
            );

        }

        telemetry.addData("Final angle", drive.imu.getAngularOrientation().firstAngle);

        if(drive.imu.getAngularOrientation().firstAngle < 0) {
            double finalOffset = Math.toRadians(180) + drive.imu.getAngularOrientation().firstAngle;
            //niggaaaaaaaaaa
            drive.turnSync(-finalOffset, drive.RF, drive.LB);
        }

        if(drive.imu.getAngularOrientation().firstAngle > 0) {
            double finalOffset = Math.toRadians(180) - drive.imu.getAngularOrientation().firstAngle;
            //niggaaaaaaaaaa
            drive.turnSync(-finalOffset, drive.RF, drive.LB);
        }



    }

}
