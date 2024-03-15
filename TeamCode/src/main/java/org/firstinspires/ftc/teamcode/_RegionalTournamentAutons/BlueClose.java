package org.firstinspires.ftc.teamcode._RegionalTournamentAutons;

import androidx.core.os.TraceKt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode._Config.HwareV2;
import org.firstinspires.ftc.teamcode._RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode._RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode._RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode._Vision._RockPipelines.RockBluePipelineRevamp;
import org.firstinspires.ftc.teamcode._Vision._UnicornPipelines.UnicornBluePipelineRevamp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "Blue", name = "BlueClose")
public class BlueClose extends LinearOpMode {
    public HwareV2 robot;

    private OpenCvWebcam webcam;
    private UnicornBluePipelineRevamp opencv = null;
    private String blueObjectPosition = "center";
    Boolean runUp = false;
    Boolean raiseArm = false;
    Boolean cycle = false;
    int park = 1;
    double ArmRotMulti = 5.74836;

    int pixRotTarget = 0;
    TrajectorySequence returnToBackDrop1;
    TrajectorySequence cycle1;
    TrajectorySequence releasePixel;

    public void changeStagePosition(int position) {

        robot.M1.setTargetPosition(position);
        robot.M2.setTargetPosition(position);
        robot.M1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.M2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (robot.M1.getCurrentPosition() < position - 150) {
            robot.M1.setPower(.4);
            robot.M2.setPower(.4);
        }

    }

    public void changeArmPosition(int position) {
        robot.armMotor.setTargetPosition(position);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    public void armToPos(int stagePos, int armPos, double power, double pixelPos, int pixRotT)
    {

        pixRotTarget = pixRotT;

        robot.M1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.M2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.M1.setPower(power);
        robot.M2.setPower(power);
        robot.armMotor.setPower(power);

        while ((Math.abs(stagePos-(int)robot.M1.getCurrentPosition())>50)) {
            if (Math.abs(stagePos-(int)robot.M1.getCurrentPosition())<500)
            {
                robot.M1.setPower(power/2);
                robot.M2.setPower(power/2);
            }
            changeStagePosition(stagePos);
            changeArmPosition(armPos);
            telemetry.addLine("Arm is doing arm things.");
            telemetry.update();
            robot.armMotor.setPower(power);
        }

    }
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new HwareV2();
        robot.initialize(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(15,38,Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    robot.autoDrop.setPosition(0.6);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(62.5,36,Math.toRadians(180)))
                .waitSeconds(.01)
                .lineToLinearHeading(new Pose2d(63,36,Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .addDisplacementMarker(() -> {
                    robot.autoDrop.setPosition(-0.6);
                    changeStagePosition(0);
                    changeArmPosition(0);
                })
                .waitSeconds(1)

                .build();


        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(21.5,50,Math.toRadians(20)))
                .addDisplacementMarker(() -> {
                    robot.autoDrop.setPosition(0.6);
                })
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(60.5,40,Math.toRadians(180)))
                .waitSeconds(.01)
                .lineToLinearHeading(new Pose2d(63,40,Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .addDisplacementMarker(() -> {
                    robot.autoDrop.setPosition(-0.8);
                    changeStagePosition(0);
                    changeArmPosition(0);
                })
                .waitSeconds(1)

                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(10,45,Math.toRadians(-30)))
                .addDisplacementMarker(() -> {
                    robot.autoDrop.setPosition(0.6);
                })
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(61.5,28,Math.toRadians(180)))
                .waitSeconds(.01)
                .lineToLinearHeading(new Pose2d(63,28,Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .addDisplacementMarker(() -> {
                    robot.autoDrop.setPosition(-0.8);
                    changeStagePosition(0);
                    changeArmPosition(0);
                })
                .waitSeconds(1)

                .build();





        robot.M1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.M2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));

        // Create the BlueElementLoco pipeline
        opencv = new UnicornBluePipelineRevamp();
        webcam.setPipeline(opencv);

        // Start streaming the camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera could not be opened");
                telemetry.update();
            }
        });

        while (!isStarted()) {
            blueObjectPosition = opencv.getLocation();

            if (gamepad2.circle) {
                cycle = !cycle;
            } else if (gamepad2.cross) {
                cycle = false;
            }


            if (gamepad2.dpad_left) {
                park = 1;
            } else if (gamepad2.dpad_up) {
                park = 2;
            } else if (gamepad2.dpad_right) {
                park = 3;
            }

            telemetry.addData("Blue Object Position", blueObjectPosition);
            telemetry.addData("\n Cycle: ", cycle);
            telemetry.addData("\n Park Location ", park);
            switch (park)
            {
                case 1:
                    telemetry.addLine("        |Back Drop|      ");
                    telemetry.addLine("  P   |__________|      ");
                    telemetry.addLine("                       ");
                    break;
                case 2:
                    telemetry.addLine("        |Back Drop|      ");
                    telemetry.addLine("        |__________|      ");
                    telemetry.addLine("                      P           ");
                    break;
                case 3:
                    telemetry.addLine("        |Back Drop|      ");
                    telemetry.addLine("        |__________|   P  ");
                    telemetry.addLine("                       ");
                    break;
            }
            telemetry.update();


            if(isStopRequested()) {return;}
        }

        robot.pixelClaw.setPosition(1);

        waitForStart();

        Pose2d cyclePoseStart = null;




        switch (blueObjectPosition){
            case "center":
                drive.followTrajectorySequence(center);
                cyclePoseStart = center.end();
                break;
            case "right":
                drive.followTrajectorySequence(right);
                cyclePoseStart = right.end();
                break;
            case "left":
                drive.followTrajectorySequence(left);
                cyclePoseStart = left.end();
                break;

        }

        TrajectorySequence waitTime = drive.trajectorySequenceBuilder(cyclePoseStart)
                .waitSeconds(.5)
                .forward(.01)
                .addDisplacementMarker(() -> {
                    runUp = false;
                })

                .build();




        drive.followTrajectorySequence(waitTime);

        robot.autoDrop.setPosition(0.5);

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(cyclePoseStart)
                .lineToLinearHeading(new Pose2d(55, 66,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(66, 66,Math.toRadians(180)))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(cyclePoseStart)
                .lineToLinearHeading(new Pose2d(60,40,Math.toRadians(180)))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(cyclePoseStart)
                .lineToLinearHeading(new Pose2d(55,16.5,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(66, 16.5,Math.toRadians(180)))
                .build();



        cycle1 = drive.trajectorySequenceBuilder(cyclePoseStart)
                .lineToLinearHeading(new Pose2d(38,16.5, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    armToPos(0, 0, 0, 0, 0);
                    robot.intake.setPower(-0.5);
                })
                .lineToLinearHeading(new Pose2d(-51,18, Math.toRadians(210)))
                .lineToLinearHeading(new Pose2d(-57,18, Math.toRadians(200)))
                .addDisplacementMarker(() -> {
                    raiseArm = true;
                    robot.intake.setPower(0.9);
                })
                .lineToLinearHeading(new Pose2d(-47,15, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-52,15, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-47,15, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-52,15, Math.toRadians(180)))
                .waitSeconds(.5)

                .lineToLinearHeading(new Pose2d(-45,16.5, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    raiseArm = false;
                    robot.intake.setPower(-0.5);
                })
                .lineToLinearHeading(new Pose2d(-40,16.5, Math.toRadians(180)))

                .addDisplacementMarker(() ->  {
                    robot.intake.setPower(0);
                    drive.followTrajectorySequenceAsync(returnToBackDrop1);
                })
                .build();


        returnToBackDrop1 = drive.trajectorySequenceBuilder(cycle1.end())

                .lineToLinearHeading(new Pose2d(38,15, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(59,40, Math.toRadians(180)))

                .addDisplacementMarker(() -> {
                    robot.intake.setPower(0);
                    runUp = true;
                })
                .lineToLinearHeading(new Pose2d(60,40, Math.toRadians(180)))
                .waitSeconds(1.5)
                .addDisplacementMarker(() -> {
                    robot.pixelClaw.setPosition(-1);
                })
                .lineToLinearHeading(new Pose2d(59.5,40, Math.toRadians(180)))
                .waitSeconds(2.5)
                .addDisplacementMarker(() -> {

                    runUp = false;
                })




                .build();


        if (cycle) {

            drive.followTrajectorySequenceAsync(cycle1);

            while (drive.isBusy()) {

                telemetry.addLine("Arm is doing arm things.");
                telemetry.update();


                if (runUp) {
                    if (raiseArm){
                        changeStagePosition(-80);
                        changeArmPosition(-50);
                    } else {
                        changeStagePosition(1400);
                        changeArmPosition(-150);
                    }
                } else {
                    changeStagePosition(0);
                    changeArmPosition(-5);
                    robot.M1.setPower(1);
                    robot.M2.setPower(1);
                    robot.armMotor.setPower(1);

                }

                double pixRotPos = robot.intake.getCurrentPosition();


                double pixRotSpeed = Math.max(Math.pow(Math.min(Math.abs(pixRotTarget - pixRotPos) / 800, 2), 1), 0.02);
                if (Math.abs(pixRotTarget - (pixRotPos + 100)) < 450)
                    robot.pixRot.setPower(0);
                else if (pixRotTarget < pixRotPos)
                    robot.pixRot.setPower(0.6 * pixRotSpeed);
                else if (pixRotTarget > pixRotPos)
                    robot.pixRot.setPower(-0.6 * pixRotSpeed);

                drive.update();
            }
        } else {

            switch (park) {
                case 1:
                    drive.followTrajectorySequence(park1);
                    break;
                case 2:
                    drive.followTrajectorySequence(park2);
                    break;
                case 3:
                    drive.followTrajectorySequence(park3);
                    break;
            }

        }



        while (!isStopRequested() && opModeIsActive());





    };





}

