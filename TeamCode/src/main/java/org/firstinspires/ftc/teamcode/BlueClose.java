package org.firstinspires.ftc.teamcode._RegionalTournamentAutons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode._Config.Hware;
import org.firstinspires.ftc.teamcode._RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode._RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode._RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode._Vision._RockPipelines.RockBluePipelineRevamp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "Blue", name = "testBlueOp")
public class BlueClose extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(90));


        TrajectorySequence leftPose = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0,48, Math.toRadians(90)))
                .build();

        waitForStart();



        drive.followTrajectorySequence(leftPose);

        while (!isStopRequested() && opModeIsActive());





    };



}

