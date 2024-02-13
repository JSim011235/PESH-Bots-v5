package org.firstinspires.ftc.teamcode._RegionalTournamentAutons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode._RoadRunner.trajectorysequence.TrajectorySequence;

@Autonomous(group = "Blue", name = "testBlueOp")
public class BlueClose extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence test = drive.trajectorySequenceBuilder(startPose)

                .lineToLinearHeading(new Pose2d(15,34,Math.toRadians(-10)))
                .waitSeconds(.5)
                .splineToSplineHeading(new Pose2d(47,23,Math.toRadians(180)), Math.toRadians(-90))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(38,12), Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(-53,12))
                .waitSeconds(2)

                .lineToConstantHeading(new Vector2d(38,12))
                .splineToConstantHeading(new Vector2d(48,30), Math.toRadians(90))

                .waitSeconds(2)

                .splineToConstantHeading(new Vector2d(38,12), Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(-53,12))
                .waitSeconds(2)

                .lineToConstantHeading(new Vector2d(38,12))
                .splineToConstantHeading(new Vector2d(48,30), Math.toRadians(90))
                .build();

        waitForStart();



        drive.followTrajectorySequence(test);

        while (!isStopRequested() && opModeIsActive());





    };



}

