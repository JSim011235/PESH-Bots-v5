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




    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HwareV2 robot = new HwareV2();
        robot.initialize(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence purplePixel = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(15,34,Math.toRadians(-10)))
                .build();

        TrajectorySequence yellowPixel = drive.trajectorySequenceBuilder(purplePixel.end())
                .splineToSplineHeading(new Pose2d(47,23,Math.toRadians(180)), Math.toRadians(-90))
                .build();

        TrajectorySequence cycle1 = drive.trajectorySequenceBuilder(yellowPixel.end())
                .splineToConstantHeading(new Vector2d(38,12), Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    robot.intake.set(1);
                })
                .lineToLinearHeading(new Pose2d(-52,16.5, Math.toRadians(200)))
                .waitSeconds(1)
                .build();

        TrajectorySequence returnToBackDrop1 = drive.trajectorySequenceBuilder(cycle1.end())
                .lineToConstantHeading(new Vector2d(38,12))
                .splineToConstantHeading(new Vector2d(56,30), Math.toRadians(90))
                .build();


        TrajectorySequence waitTIme = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(.5)
                .build();


        TrajectorySequence test = drive.trajectorySequenceBuilder(startPose)

                //.lineToLinearHeading(new Pose2d(15,34,Math.toRadians(-10)))
                .waitSeconds(.5)
                //.splineToSplineHeading(new Pose2d(47,23,Math.toRadians(180)), Math.toRadians(-90))
                .waitSeconds(1)
              //  .splineToConstantHeading(new Vector2d(38,12), Math.toRadians(-90))
                //.lineToConstantHeading(new Vector2d(-53,12))
                .waitSeconds(2)

                //.lineToConstantHeading(new Vector2d(38,12))
                //.splineToConstantHeading(new Vector2d(48,30), Math.toRadians(90))

                .waitSeconds(2)

              //  .splineToConstantHeading(new Vector2d(38,12), Math.toRadians(-90))
              //  .lineToConstantHeading(new Vector2d(-53,12))
                .waitSeconds(2)

                .lineToConstantHeading(new Vector2d(38,12))
                .splineToConstantHeading(new Vector2d(48,30), Math.toRadians(90))
                .build();
        robot.armMotor.resetEncoder();

        waitForStart();



        drive.followTrajectorySequence(purplePixel);
        robot.autoDrop.setPosition(0.8);
        drive.followTrajectorySequence(yellowPixel);
        robot.autoDrop.setPosition(-0.8);
        drive.followTrajectorySequence(waitTIme);
        drive.followTrajectorySequence(cycle1);
        drive.followTrajectorySequence(returnToBackDrop1);
        robot.M1.setTargetPosition(2500);
        robot.M2.setTargetPosition(2500);
        drive.followTrajectorySequence(waitTIme);
        robot.pixelClaw.setPosition(-1);


        while (!isStopRequested() && opModeIsActive());





    };





}

