package org.firstinspires.ftc.teamcode._RegionalTournamentAutons;

import androidx.core.os.TraceKt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Config.HwareV2;
import org.firstinspires.ftc.teamcode._RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode._RoadRunner.trajectorysequence.TrajectorySequence;

@Autonomous(group = "Blue", name = "testBlueOp")
public class BlueClose extends LinearOpMode {
    public HwareV2 robot;


    public void changeStagePosition(int position) {
        robot.M1.setTargetPosition(position);
        robot.M2.setTargetPosition(position);
        robot.M1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.M2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void changeArmPosition(int position) {
        robot.armMotor.setTargetPosition(position);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new HwareV2();
        robot.initialize(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence purplePixel = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(15,38,Math.toRadians(-10)))
                .build();

        TrajectorySequence yellowPixel = drive.trajectorySequenceBuilder(purplePixel.end())
                .lineToLinearHeading(new Pose2d(63,36,Math.toRadians(180)))
                .build();

        TrajectorySequence cycle1 = drive.trajectorySequenceBuilder(yellowPixel.end())
                .lineToLinearHeading(new Pose2d(38,16.5, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    robot.intake.set(1);
                })
                .lineToLinearHeading(new Pose2d(-51,18, Math.toRadians(200)))
                .lineToLinearHeading(new Pose2d(-53,18, Math.toRadians(200)))
                .lineToLinearHeading(new Pose2d(-53,17, Math.toRadians(190)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-40,16.5, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    robot.intake.set(-0.5);
                })
                .build();

        TrajectorySequence returnToBackDrop1 = drive.trajectorySequenceBuilder(cycle1.end())
                .lineToLinearHeading(new Pose2d(38,14, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    robot.intake.set(0);
                })
                .lineToLinearHeading(new Pose2d(62.5,40, Math.toRadians(180)))
                .build();


        TrajectorySequence waitTime = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(.5)
                .build();
        TrajectorySequence tinyWait = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(.2)
                .build();

        robot.M1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.M2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();


        robot.pixelClaw.setPosition(1);
        drive.followTrajectorySequence(purplePixel);
        robot.autoDrop.setPosition(0.6);
        drive.followTrajectorySequence(tinyWait);
        drive.followTrajectorySequence(yellowPixel);
        robot.autoDrop.setPosition(-0.8);
        drive.followTrajectorySequence(waitTime);
        robot.autoDrop.setPosition(0.6);
        changeStagePosition(0);
        changeArmPosition(0);
        drive.followTrajectorySequence(cycle1);
        drive.followTrajectorySequence(returnToBackDrop1);
        changeStagePosition(2500);
        drive.followTrajectorySequence(waitTime);
        drive.followTrajectorySequence(waitTime);
        robot.pixelClaw.setPosition(0);


        while (!isStopRequested() && opModeIsActive());





    };





}

