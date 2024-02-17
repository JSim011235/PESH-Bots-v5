package org.firstinspires.ftc.teamcode._Config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HwareV2 {
    // input motors exactly as shown below
    public Motor frontRight;
    public Motor frontLeft;
    public Motor backRight;
    public Motor backLeft;

    public DcMotor M1, M2, armMotor;
    public MotorEx M1x, M2x;
    public DcMotor intake=  null;

    public Servo pixelClaw = null;
    public Servo airplaneLaunch = null;
    public DistanceSensor distances = null;
    public ColorRangeSensor pixColor = null;
    public CRServo pixRot = null;

    public Servo autoDrop = null;

    HardwareMap hardwareMap;

    public HwareV2() {
        hardwareMap = null;
    }

    public void initialize(HardwareMap hwMap) {
        hardwareMap = hwMap;


        // Drive Controls (Chassis)
        frontRight = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_435);
        frontLeft = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_435);
        backRight = new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_435);
        backLeft = new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_435);

        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

//        frontRight.setInverted(true);
//        backRight.setInverted(true);

        // Arm Controls (Main-stage)
        M1 = hardwareMap.get(DcMotor.class, "MainStage1");
        M2 = hardwareMap.get(DcMotor.class, "MainStage2");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        M2.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        M1x = new MotorEx(hardwareMap, "MainStage1", Motor.GoBILDA.RPM_117);
        M2x = new MotorEx(hardwareMap, "MainStage2", Motor.GoBILDA.RPM_117);

        M2x.setInverted(true);


        intake = hardwareMap.get(DcMotor.class, "intake");


        // Servos
        // Servos
        pixelClaw = hardwareMap.get(Servo.class, "pixelClaw");
        airplaneLaunch = hardwareMap.get(Servo.class, "airplaneLaunch");
        pixRot = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, "pixRot");
        autoDrop = hardwareMap.get(Servo.class, "autoDrop");

        // Sensors
        distances = hardwareMap.get(DistanceSensor.class, "distance");
        pixColor = hardwareMap.get(ColorRangeSensor.class, "pixCol");

    }
}
