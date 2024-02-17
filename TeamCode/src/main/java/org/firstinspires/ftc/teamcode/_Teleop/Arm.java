package org.firstinspires.ftc.teamcode._Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode._Config.HwareV2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
@TeleOp
public class Arm extends LinearOpMode {
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
    public static double kP =  0;
    public static double kI =  0;
    public static double kD =  0;
    public static double kS =  0;
    public static double kV =  0;
    public static double kA =  0;

    public static int target = 0;
    public static int min = 0;
    public static int max = 1200;
    public static int speed = 1;

    public static double arm_max_accel =  0;
    public static double arm_max_velocity =  0;

    private static double curTime = 0;

    public static int tol = 1;


//    public void changeStagePosition(int position) {
//        robot.M1.setTargetPosition(position);
//        robot.M2.setTargetPosition(position);
//        robot.M1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.M2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    public void changeArmPosition(int position) {
//        robot.armMotor.setTargetPosition(position);
//        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }


    public double arm_profile(int target_pos, int current_pos, double current_velocity, double time_diff) {
        int direction_multiplier = 1;
        int position_error = target_pos-current_pos;
        if (position_error < 0) direction_multiplier = -1;

        double output_velocity = 0;

        //if maximum speed has not been reached
        if (arm_max_velocity > Math.abs(current_velocity)) {
            output_velocity = current_velocity + direction_multiplier * arm_max_accel * time_diff;
        }
        //#if maximum speed has been reached, stay there for now
        else {
            output_velocity = arm_max_velocity;
        }
        //if we are close enough to the object to begin slowing down
        if (position_error <= (output_velocity * output_velocity) / (2 * arm_max_accel)) {
            output_velocity = current_velocity - direction_multiplier * arm_max_accel * time_diff;
        }
        if (Math.abs(position_error)<tol)
            output_velocity = 0;
        return output_velocity*direction_multiplier;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        double time_diff = time - curTime;
        curTime = time;

        // input motors exactly as shown below
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // input motors exactly as shown below
        robot.initialize(hardwareMap);

        drive = new MecanumDrive(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);
        driverOp = new GamepadEx(gamepad1);
        armOp = new GamepadEx(gamepad2);

        // Initial Positions
        robot.pixelClaw.setPosition(1);
        PIDFController pidf = new PIDFController(17, 0 ,3 ,2);
        boolean auto = false;
        boolean swap = false;

        waitForStart();

        while(opModeIsActive()) {
            time_diff = time - curTime;
            curTime = time;

            if (gamepad2.triangle) {if (swap) {
                auto = !auto;
                swap = false;
            }}
            else
                swap = true;
            pidf = new PIDFController(0,0,0,0);
            if (auto) {
                if (target < min)
                    speed = Math.abs(speed);
                else if (target > max)
                    speed = -Math.abs(speed);
                target += speed;
                pidf.setSetPoint(target);
            } else
                pidf.setSetPoint(target * gamepad2.left_stick_y);

            drive.driveRobotCentric(
                    -driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    -driverOp.getRightX()
            );

//            double output = feedforward.calculate(pidf.getSetPoint()-robot.M1.getCurrentPosition(),
//                    arm_max_velocity,arm_max_accel);
            double output = arm_profile(((int) pidf.getSetPoint()),robot.M1x.getCurrentPosition(), robot.M1x.getCorrectedVelocity(), 0.02);
            robot.M1x.setVelocity(output);
            robot.M2x.setVelocity(output);

            robot.M1x.setRunMode(Motor.RunMode.VelocityControl);
            robot.M2x.setRunMode(Motor.RunMode.VelocityControl);

            robot.M1x.setVeloCoefficients(kP, kI, kD);
            robot.M2x.setVeloCoefficients(kP, kI, kD);
            robot.M1x.setFeedforwardCoefficients(kS, kV, kA);
            robot.M2x.setFeedforwardCoefficients(kS, kV, kA);


//            telemetry.addData("p", p);
//            telemetry.addData("i", i);
//            telemetry.addData("d", d);
//            telemetry.addData("f", f);
            telemetry.addData("target", pidf.getSetPoint());
            telemetry.addData("current", robot.M1x.getCurrentPosition());
            telemetry.addData("auto", auto);
            telemetry.addData("speed", speed);
            telemetry.addData("min", min);
            telemetry.addData("max", max);
            telemetry.addData("targ_velo", output);
            telemetry.addData("Curr_velo", robot.M1x.getCorrectedVelocity());
            telemetry.addData("time", time_diff);

            telemetry.update();
        }

    }}

