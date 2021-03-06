package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.Trajectories;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.ExpansionHubEx;

import kotlin.Unit;

@Autonomous
public class odoTest extends LinearOpMode {

    String SkyStoneLocation;

    public void runOpMode(){

        SampleMecanumDriveREVOptimized robot = new SampleMecanumDriveREVOptimized(hardwareMap, true);

        StandardThreeWheelLocalizer odometery = new StandardThreeWheelLocalizer(hardwareMap);

        robot.setPoseEstimate(new Pose2d(0,0,0));

        double voltage = robot.hub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
        double voltage2 = robot.hub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);

        final double offset = 0;

        Trajectories traj = new Trajectories();

        ConstantInterpolator FACING_LZ = new ConstantInterpolator(Math.toRadians(0));

        //TODO: Add servo init positions

        telemetry.addLine("Ready");
        telemetry.addData("Voltage 1", voltage);
        telemetry.addData("Voltage 2", voltage2);


        while(!isStarted()){

            SkyStoneLocation = robot.detectSkystone();
            telemetry.addData("Skystone Location", SkyStoneLocation);
            telemetry.update();

            if(opModeIsActive()){
                break;
            }

        }

        robot.followTrajectorySync(
                robot.trajectoryBuilder()
                .lineTo(new Vector2d(48, 24), new ConstantInterpolator(0))
                .lineTo(new Vector2d(80, 0), new ConstantInterpolator(0))
                .build()
        );


        robot.poseCorrect(robot, 80, 0);
        robot.killAll();

        robot.relayPose(telemetry, robot);

        sleep(500000);



    }

}
