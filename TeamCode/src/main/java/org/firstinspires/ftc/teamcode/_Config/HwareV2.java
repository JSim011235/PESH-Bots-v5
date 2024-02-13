package org.firstinspires.ftc.teamcode._Config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HwareV2 {
    // input motors exactly as shown below
    public Motor frontRight;
    public Motor frontLeft;
    public Motor backRight;
    public Motor backLeft;

    public MotorEx M1, M2, armMotor;
    public Motor intake;
    public ServoEx pixelClaw = null;
    public ServoEx airplaneLaunch = null;
    public DistanceSensor distances = null;
    public ColorRangeSensor pixColor = null;
    public CRServo pixRot = null;

    public ServoEx autoDrop = null;

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

        // Arm Controls (Main-stage)
        M1 = new MotorEx(hardwareMap, "MainStage1", MotorEx.GoBILDA.RPM_117);
        M2 = new MotorEx(hardwareMap, "MainStage2", MotorEx.GoBILDA.RPM_117);
        armMotor = new MotorEx(hardwareMap, "armMotor", MotorEx.GoBILDA.RPM_117);
        M1.setInverted(true);
        intake = new MotorEx(hardwareMap, "intake", MotorEx.GoBILDA.RPM_435);

        // Servos
        pixelClaw = new SimpleServo(hardwareMap,"pixelClaw", 0,180);
        airplaneLaunch = new SimpleServo(hardwareMap,"airplaneLaunch", 0,180);
        pixRot = new CRServo(hardwareMap, "pixRot");
        autoDrop = new SimpleServo(hardwareMap,"autoDrop", 0,180);


    }
}
