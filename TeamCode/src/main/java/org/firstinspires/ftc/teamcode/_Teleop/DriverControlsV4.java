package org.firstinspires.ftc.teamcode._Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OutdatedVersions.Hware;
import org.firstinspires.ftc.teamcode._Config.HwareV2;

@Config
@TeleOp
public class DriverControlsV4 extends LinearOpMode {
    private Motor fL, fR, bL, bR;
    private MotorEx M1, M2;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private GamepadEx armOp;


    double SPEEDCONTROL = 1;
    double TURNCONTROL = 1;

    private HwareV2 robot = new HwareV2();
    private double x, y, xy, main1, main2, armEnc, pixRotPos;
    double THRESHOLD = 0.10, DEGREE = 0;
    double Sa, Sb, Sc, Aa, Ab, Ac, targPos, bbDist;
    boolean RUMBLE = true, mainManual = true, armManual = true, pixRotManual = false, launchMode = false;
    int pixRotTarget = 0, armDownCount = 0;
    String armState = "Not Set";

    double distanceAlign = 100;
    boolean alignOn = false;

    final double ArmRotMulti = -5.74836;


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

        // input motors exactly as shown below
        robot.initialize(hardwareMap);

        drive = new MecanumDrive(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);
        driverOp = new GamepadEx(gamepad1);
        armOp = new GamepadEx(gamepad2);

        // Reset Positions
        robot.M1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.M2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initial Positions
        robot.pixelClaw.setPosition(1);
        robot.airplaneLaunch.setPosition(0.7);

        waitForStart();

        while(opModeIsActive()) {

            // ARKIN CONTROLS

            x = -driverOp.getLeftX() * SPEEDCONTROL;
            y = driverOp.getLeftY() * SPEEDCONTROL;
            xy = -driverOp.getRightX() * TURNCONTROL;

            y = gamepad1.square ? 0 : y; //Perfect straight
            x = gamepad1.cross ? 0 : x; //Perfect straight

            if (gamepad1.left_trigger > 0.5) {SPEEDCONTROL = .4;TURNCONTROL=.4;} else if(gamepad1.right_trigger>0.5) {TURNCONTROL = 0.4; SPEEDCONTROL = 1;} else {SPEEDCONTROL=1; TURNCONTROL=1;}

            drive.driveRobotCentric(
                    x,
                    y,
                    xy
            );



            /* Gamepad 2 */
            main1 = robot.M1.getCurrentPosition();
            main2 = robot.M2.getCurrentPosition();
            armEnc = robot.armMotor.getCurrentPosition();
            pixRotPos = robot.intake.getCurrentPosition();

            robot.autoDrop.setPosition(gamepad2.options ? -.5 : 0.5);

            if (gamepad2.cross) {
                // Arm Down
                changeStagePosition(0);
                changeArmPosition(-15);
                pixRotTarget = 0;
                mainManual = armManual = pixRotManual = false;
                armState = "Down";
            }
            else if (gamepad2.triangle) {
                // Scoring Max
                changeStagePosition(1560);
                changeArmPosition(-480);
                pixRotTarget = (int)(robot.armMotor.getTargetPosition() * ArmRotMulti)-500;
                mainManual = armManual = pixRotManual = false;
                armState = "Scoring Max";
            } else if (gamepad2.square){
                //Scoring Low
                changeStagePosition(1600);
                changeArmPosition(-180);
                pixRotTarget = -50;
                mainManual = armManual = pixRotManual = false;
                armState = "Scoring Low";
            } else if (gamepad2.circle && !gamepad2.options) {
                double Sa, Sb, Sc, Aa, Ab, Ac, MSA, AA;
                Sa = Math.sqrt(Math.pow(200,2) + Math.pow(x,2));
                Sb = x + 111.843;
                Sc = 223.686;
                Aa = Math.acos((Math.pow(Sa,2) + Math.pow(Sc,2) - Math.pow(Sb,2)) / (2 * Sa * Sc));
                Ab = Math.asin(Math.sin(Aa) * Sc / Sb);
                Ac = Math.asin(Math.sin(Aa) * Sa / 508);
                MSA = 180 - Math.toDegrees(Aa + Ab + Ac);
                AA = -MSA;
                // Scoring Mid
                changeStagePosition(1400);
                changeArmPosition(-230);
                pixRotTarget = (int)(robot.armMotor.getTargetPosition() * ArmRotMulti)-500;
                mainManual = armManual = pixRotManual = false;
                armState = "Scoring Mid";
            }
            else if (gamepad2.dpad_up) {}
            else if (gamepad2.dpad_right) {
                // Forward Climb
                changeStagePosition(1015);
                changeArmPosition(-308);
                pixRotTarget = (int)(robot.armMotor.getTargetPosition() * ArmRotMulti)-500;
                mainManual = armManual = pixRotManual = false;
                armState = "Forward Climb";
            }
            else if (gamepad2.dpad_left) {
                // Backward Climb
                changeStagePosition(1750);
                changeArmPosition(257);
                pixRotTarget = (int)(robot.armMotor.getTargetPosition() * ArmRotMulti)-500;
                mainManual = armManual = pixRotManual = false;
                armState = "Backward Climb";
            }
            else if (gamepad2.dpad_down) {
                // Partially Down
                changeStagePosition(250);
                changeArmPosition(-15);
                pixRotTarget = (int)(robot.armMotor.getTargetPosition() * ArmRotMulti)-500;
                mainManual = armManual = pixRotManual = false;
                armState = "Partially Down";
            }

            if (gamepad2.left_stick_y != 0 && !mainManual) {
                robot.M1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.M2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mainManual = true;
            }

            if (gamepad2.right_stick_y != 0 && !armManual) {
                robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armManual = true;
            }

            pixRotManual = gamepad2.right_stick_button;

            if (mainManual && armManual)
                if (pixRotManual) armState = "Full Manual";
                else armState = "Manual";

            if (mainManual) {
                robot.M1.setPower(-gamepad2.left_stick_y);
                robot.M2.setPower(-gamepad2.left_stick_y);
            }
            else {
                if (Math.abs(robot.M1.getTargetPosition() - main1) < 400) {
                    robot.M1.setPower(.5);
                    robot.M2.setPower(.5);
                }
                else {
                    robot.M1.setPower(1);
                    robot.M2.setPower(1);
                }
            }

            if (armManual) {
                robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (armEnc < -700 && gamepad2.right_stick_y < 0) robot.armMotor.setPower(0);
                else if (armEnc > 900 && gamepad2.right_stick_y > 0) robot.armMotor.setPower(0);
                else robot.armMotor.setPower(gamepad2.right_stick_y);
            } else {
                if (Math.abs(robot.armMotor.getTargetPosition() - armEnc) < 100) robot.armMotor.setPower(.6);
                else robot.armMotor.setPower(1);
            }

            // Pixel Rotation Control
            if (pixRotManual) pixRotTarget += (int) (gamepad2.right_stick_x * 200);
            else if(armManual) pixRotTarget = (int)(armEnc * ArmRotMulti) -500;

            double pixRotSpeed = Math.max(Math.pow(Math.min(Math.abs(pixRotTarget - pixRotPos) / 800, 2), 1), 0.02);
            if (Math.abs(pixRotTarget - (pixRotPos+100)) < 650)
                robot.pixRot.setPower(0);
            else if (pixRotTarget < pixRotPos)
                robot.pixRot.setPower(0.6 * pixRotSpeed);
            else if (pixRotTarget > pixRotPos)
                robot.pixRot.setPower(-0.6 * pixRotSpeed);

            if (gamepad2.touchpad) {
                robot.M1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.M2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (gamepad2.share) {
                robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                gamepad2.rumbleBlips(3);
            }

            robot.intake.setPower(gamepad2.left_trigger != 0 ? gamepad2.left_trigger : -gamepad2.right_trigger);

            if (gamepad2.left_trigger != 0) {
                if (armState == "Down") {
                    changeStagePosition(-40);
                    changeArmPosition(-55);
                }
                if (robot.pixColor.getDistance(DistanceUnit.MM) < 10) {
                    gamepad2.rumbleBlips(1);
                    gamepad1.rumbleBlips(1);
                }
            }

            robot.pixelClaw.setPosition(gamepad2.right_bumper ? 0 : 1);

            if (gamepad2.dpad_up && !(gamepad1.left_bumper && gamepad1.right_bumper)) {
                launchMode = true;
                if (gamepad2.left_bumper) {
                    robot.airplaneLaunch.setPosition(-1);
                    if (!RUMBLE) {
                        gamepad2.rumbleBlips(1);
                        RUMBLE = true;
                    }
                } else if (RUMBLE) {
                    gamepad2.rumbleBlips(1);
                    RUMBLE = false;
                }

                if (!gamepad1.atRest()) {
                    gamepad1.rumble(1000);
                }
            } else if(launchMode){
                launchMode = false;
                robot.airplaneLaunch.setPosition(0.7);
                RUMBLE = true;
                if (gamepad1.isRumbling()) {
                    gamepad1.stopRumble();
                }
            }



            // Telemetry Data
            telemetry.clearAll();
            if (!launchMode) {
                telemetry.addLine("Current Speed is: " + (int) (SPEEDCONTROL * 100) + "%"); // Overall Speed
                telemetry.addLine("Current Turn Speed is: " + (int) (TURNCONTROL * 100) + "%"); // Backup Speed
            } else {
                telemetry.addLine("Ready to launch");
            }
            telemetry.addLine("-----------------------------------------------------------------------------");
            telemetry.addLine("Main Stage: " + main1 + ", " + main2);
            telemetry.addLine("Arm Enc: " + armEnc);
            telemetry.addLine("PixRot Current: " + pixRotPos + " PixRot Target: " + pixRotTarget);
            telemetry.addLine("Arm Controls: Main " + (mainManual ? "Manual" : "Auto") + "; Arm " + (armManual ? "Manual" : "Auto") + "; Pixel Rotation " + (pixRotManual ? "Manual" : "Auto"));
            telemetry.addLine("Arm State: " + armState);
            telemetry.addLine("Distance is " + robot.distances.getDistance(DistanceUnit.MM) + "mm");
            telemetry.addLine("Pix Dist is " + robot.pixColor.getDistance(DistanceUnit.MM) + "mm");
//            telemetry.addLine("Arm degree is " + DEGREE);
//            telemetry.addLine("Target position is " + targPos);

            telemetry.update();
        }



    }
}
