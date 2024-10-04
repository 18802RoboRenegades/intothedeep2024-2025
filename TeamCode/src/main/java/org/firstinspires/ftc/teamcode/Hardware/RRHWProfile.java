package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class RRHWProfile {


    public CSAutoParams params;

    /*
     * Hardware devices
     */

//    public RevIMU imu = null;

    public HardwareMap hwMap;

    /*
     * Declare Odometry hardware
     */

    /* Constructor */
    public RRHWProfile(CSAutoParams myParams) {
        params = myParams;
    }

    public void init(HardwareMap ahwMap) {

        params = new CSAutoParams();
        hwMap = ahwMap;

        /* Webcam device will go here */
//        webcam = hwMap.get(WebcamName.class, "Webcam 1");
    }


    /**
     * The RunMode of the motor.
     */
    public enum RunMode {
        VelocityControl, PositionControl, RawPower
    }
}
