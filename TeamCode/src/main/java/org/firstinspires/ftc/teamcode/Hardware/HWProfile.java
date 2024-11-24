package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class HWProfile {


    /*
     * Constants
     */

    public final double PIVOT_SPEED = 0.5;
    public final double COUNTS_PER_ROTATION = 28;
    public final double GB_COUNTS_PER_ROTATION = 28;    // goBilda encoder value
    public final double MIN_PIDROTATE_POWER = 0.10;


    /*
     *  Constants & variables for wheel parameters
     */

    public final double DRIVE_TICKS_PER_INCH = 32;
    public final double STRAFE_FACTOR = 0.9;

    public final double MAX_DRIVING_POWER = 1;
    public final double MIN_STRAFE_POWER = 0.35;

    public final double INTAKE_CLAW_OPEN = 0.9;
    public final double INTAKE_CLAW_CLOSE = 0.7;

    // Intake angle servo constants
    public final double INTAKE_ANGLE_GRAB_SPECIMEN = 0.03;
    public final double INTAKE_ANGLE_GRAB_SAMPLE = 0.5;
    public final double INTAKE_ANGLE_SCORE_SPECIMEN = 0.5;
    public final double INTAKE_ANGLE_SCORE_SAMPLE = 0.5;

    // Arm Angle motor constants
    public final int ARM_ANGLE_GRAB_SPECIMEN = 0;
    public final int ARM_ANGLE_SCORE_SPECIMEN = -1000;
    public final int ARM_ANGLE_SCORE_HIGH_BASKET = -1310;

    public final int ARM_LENGTH_RESET = 0;
    public final int ARM_LENGTH_SAFE = 400;
    public final int ARM_LENGTH_SCORE_SPECIMEN = 400;
    public final int ARM_LENGTH_SCORE_HIGH_BASKET = 3790;

    /*
     * Hardware devices
     */

    public RevIMU imu = null;

    public DcMotorEx motorLF;
    public DcMotorEx motorLR;
    public DcMotorEx motorRF;
    public DcMotorEx motorRR;
    public DcMotorEx motorArmAngle;
    public DcMotorEx motorArmLength;
    public Servo servoIntakeAngle;
    //public CRServo servoIntake;
    public Servo servoIntake;

//    public MecanumDrive mecanum = null;

    HardwareMap hwMap;

    /*
     * Declare Odometry hardware
     */

    /* Constructor */
    public HWProfile() {
    }

    public void init(HardwareMap ahwMap, boolean teleop) {

        hwMap = ahwMap;

        if(teleop) {
            motorLF = ahwMap.get(DcMotorEx.class, "motorLF");
            motorLF.setDirection(DcMotor.Direction.FORWARD);
            motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorLF.setPower(0);
            motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            motorLR = ahwMap.get(DcMotorEx.class, "motorLR");
            motorLR.setDirection(DcMotor.Direction.REVERSE);
            motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorLR.setPower(0);
            motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorRF = ahwMap.get(DcMotorEx.class, "motorRF");
            motorRF.setDirection(DcMotor.Direction.REVERSE);
            motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorRF.setPower(0);
            motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorRR = ahwMap.get(DcMotorEx.class, "motorRR");
            motorRR.setDirection(DcMotor.Direction.FORWARD);
            motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorRR.setPower(0);
            motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            imu = new RevIMU(ahwMap);
            imu.init();
        }

        motorArmAngle = ahwMap.get(DcMotorEx.class,"motorArmAngle");
        motorArmAngle.setDirection(DcMotor.Direction.FORWARD);
        motorArmAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmAngle.setTargetPosition(0);
        motorArmAngle.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorArmAngle.setPower(0);
        motorArmAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorArmLength = ahwMap.get(DcMotorEx.class,"motorArmLength");
        motorArmLength.setDirection(DcMotor.Direction.FORWARD);
        motorArmLength.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmLength.setTargetPosition(0);
        motorArmLength.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorArmLength.setPower(0);
        motorArmLength.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoIntakeAngle = ahwMap.get(Servo.class,"servoIntakeAngle");

        servoIntake =ahwMap.get(Servo.class,"servoIntake");

    }
}
