package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
public class AprilTagsTest extends OpMode {

    private Limelight3A limelight;
    private IMU imu;
    @Override
    public void init(){
    limelight=hardwareMap.get(Limelight3A.class,"limelight");
    limelight.pipelineSwitch(8); //april tag #20 blue goal
         limelight.start(); //can go in the start below, but may be a delay
        imu=hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    }

    @Override
    public void start(){
        limelight.start(); //can go in the start below, but may be a delay
    }

    @Override
    public void loop(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult=limelight.getLatestResult();

        if(llResult != null && llResult.isValid()){
            Pose3D botPose =llResult.getBotpose_MT2();
            telemetry.addData("Target x:", llResult.getTx());
            telemetry.addData("Target y:", llResult.getTy());
            telemetry.addData("Target area:", llResult.getTa());
            telemetry.addData("Bot Pose:", botPose.toString());
            telemetry.addData("Yaw:", botPose.getOrientation().getYaw());

        }
    }
}
