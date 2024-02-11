package org.firstinspires.ftc.teamcode._Config;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class Hware {
    public DcMotor frontRight = null;
    public DcMotor frontLeft = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;

    public DcMotor MainStage1 = null;

    public DcMotor MainStage2 = null;

    public DcMotor armMotor = null;

    public DcMotor intake=  null;

    public Servo pixelClaw = null;
    public Servo airplaneLaunch = null;
    public DistanceSensor distances = null;
    public ColorRangeSensor pixColor = null;
    public CRServo pixRot = null;

    public Servo autoDrop = null;

    HardwareMap hardwareMap;

    public Hware() {
        hardwareMap = null;
    }

    public void initialize(HardwareMap hwMap)
    {
        hardwareMap = hwMap;


        // Driver Controls
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        distances = hardwareMap.get(DistanceSensor.class, "distance");
        pixColor = hardwareMap.get(ColorRangeSensor.class, "pixCol");

        // Arms
        MainStage1 = hardwareMap.get(DcMotor.class, "MainStage1");
        MainStage2 = hardwareMap.get(DcMotor.class, "MainStage2");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Servos
        pixelClaw = hardwareMap.get(Servo.class, "pixelClaw");
        airplaneLaunch = hardwareMap.get(Servo.class, "airplaneLaunch");
        pixRot = hardwareMap.get(CRServo.class, "pixRot");
        autoDrop = hardwareMap.get(Servo.class, "autoDrop");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        pixRot.setDirection(CRServo.Direction.FORWARD);
        autoDrop.setDirection(Servo.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MainStage2.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        

        MainStage1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MainStage2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        MainStage1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MainStage2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
