package org.firstinspires.ftc.teamcode._Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode._Config.HwareV2;

@Config
@TeleOp
public class ArmTuning extends LinearOpMode {
    private Motor fL, fR, bL, bR;
    private MotorEx M1, M2;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    public static double kS =  0;
    public static double kCos =  0;
    public static double kV =  0;
    public static double kA =  0;
    public static double kp =  0;
    public static double ki =  0;
    public static double kd =  0;
    public static double kf =  0;
    public static int target = 0;
    public static int min = 0;
    public static int max = 1200;
    public static int speed = 1;

    public static double arm_max_accel =  0;
    public static double arm_max_velocity =  0;

    private static double curTime = 0;

    public double arm_profile(int target_pos, int current_pos, double current_velocity, double time_diff) {
        int direction_multiplier = 1;
        int position_error = target_pos-current_pos;
        if (position_error < 0) direction_multiplier = -1;

        double output_velocity = 0;
        double output_acceleration = 0;

        //if maximum speed has not been reached
        if (arm_max_velocity > Math.abs(current_velocity)) {
            output_velocity = current_velocity + direction_multiplier * arm_max_accel * time_diff;
            output_acceleration = arm_max_accel;
        }
        //#if maximum speed has been reached, stay there for now
        else {
            output_velocity = arm_max_velocity;
            output_acceleration = 0;
        }
        //if we are close enough to the object to begin slowing down
        if (position_error <= (output_velocity * output_velocity) / (2 * arm_max_accel)) {
            output_velocity = current_velocity - direction_multiplier * arm_max_accel * time_diff;
            output_acceleration = -arm_max_accel;
        }
//        double[] out = {output_velocity,output_acceleration};
        return output_velocity;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        double time_diff = time - curTime;
        curTime = time;

        // input motors exactly as shown below
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        HwareV2 robot = new HwareV2();
        robot.initialize(hardwareMap);


        drive = new MecanumDrive(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);
        driverOp = new GamepadEx(gamepad1);

        PIDFController pidf = new PIDFController(17, 0 ,3 ,2);
        boolean auto = false;
        boolean swap = false;

        PController pController = new PController(17);
        pController.setSetPoint(1200);
        ArmFeedforward feedforward = new ArmFeedforward(kS, kCos, kV, kA);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad2.triangle) {if (swap) {
                auto = !auto;
                swap = false;
            }}
            else
                swap = true;
            feedforward = new ArmFeedforward(kS, kCos, kV, kA);
            pidf = new PIDFController(kp,ki,kd,kf);
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

            double targetVelocity = pidf.calculate(robot.M1.getCurrentPosition());
            double output = feedforward.calculate(targetVelocity,
                    arm_max_velocity,arm_max_accel);
//            double output = arm_profile(((int) pidf.getSetPoint()),robot.M1.getCurrentPosition(), robot.M1.getCorrectedVelocity(), time_diff);
            robot.M1.set(output);
            robot.M2.set(output);

//            telemetry.addData("p", p);
//            telemetry.addData("i", i);
//            telemetry.addData("d", d);
//            telemetry.addData("f", f);
            telemetry.addData("target", pidf.getSetPoint());
            telemetry.addData("current", robot.M1.getCurrentPosition());
            telemetry.addData("auto", auto);
            telemetry.addData("speed", speed);
            telemetry.addData("min", min);
            telemetry.addData("max", max);
            telemetry.update();


        }
        }

    }

