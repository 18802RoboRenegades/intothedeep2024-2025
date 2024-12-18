package org.firstinspires.ftc.teamcode.Libs;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class DriveMecanumFTCLib {

    private final HWProfile robot;
    public LinearOpMode opMode;

    FtcDashboard dashboard;


    /*
     * Constructor method
     */
    public DriveMecanumFTCLib(HWProfile myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }   // close DriveMecanum constructor Method





    /* #########################################################################################
       #########################################################################################
       ################################  DRIVE METHODS #########################################
       #########################################################################################
       #########################################################################################
     */

    /******************************************************************************************
     * Method:      ftclibDrive
     * Function:    Robot drives the direction of the heading, at the power provided,
     *              for the distance provided
     * Note:        This function is intended to work at 0, 90, 180, and -90 headings
     * Parameters:
     * @param heading   - Direction robot should drive
     * @param distance  - Distance in Inches to drive
     */


    /* #########################################################################################
       #########################################################################################
       ################################  MECHANISM METHODS #####################################
       #########################################################################################
       #########################################################################################
     */

    public void StrafeDrive(double drive, double turn, double strafe) {

        double leftPower    = -Range.clip(drive, -robot.MAX_DRIVING_POWER, robot.MAX_DRIVING_POWER);
        double rightPower   = -Range.clip(drive, -robot.MAX_DRIVING_POWER, robot.MAX_DRIVING_POWER);
        double strafePower = Range.clip(-strafe, -robot.MAX_DRIVING_POWER, robot.MAX_DRIVING_POWER);

        robot.motorLF.setPower(leftPower - turn + strafePower);
        robot.motorLR.setPower(leftPower - turn - strafePower);
        robot.motorRF.setPower(rightPower + turn - strafePower);
        robot.motorRR.setPower(rightPower + turn + strafePower);

//        robot.motorLF.setPower(turn);
//        robot.motorLR.setPower(turn);
//        robot.motorRF.setPower(-turn);
//        robot.motorRR.setPower(-turn);
    }


    /******************************************************************************************
     * Method:  getZAngle()
     ******************************************************************************************/
    public double getZAngle(){
        return (-robot.imu.getAbsoluteHeading());
    }   // close getZAngle method

    /******************************************************************************************
     * Method:  getZAngleRadians()
     ******************************************************************************************/
    public double getZAngleRadians(){
        return (Math.toRadians(-robot.imu.getAbsoluteHeading()));
    }   // close getZAngle method

    /* #########################################################################################
       #########################################################################################
       ################################  CLASS CALCULATIONS ####################################
       #########################################################################################
       #########################################################################################
     */


    /*******************************************************************************************
     * Method gyro360
     *  - Causes the Gyro to behave in 360 mode instead of 180 degree mode
     *******************************************************************************************/
    public double gyro360(double targetAngle){
        double currentZ = getZAngle();
        double rotationalAngle;

        if (targetAngle > 0){
            if ((currentZ >= 0) && (currentZ <= 180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = 180 + (180 + currentZ);
            }// end if(currentZ <=0) - else
        } else {
            if ((currentZ <= 0) && (currentZ >= -180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = -180 - (180 - currentZ);
            }   // end if(currentZ <=0) - else
        }   // end if(targetAngle >0)-else

        return rotationalAngle;
    }   // end method gyro360

    public double angleToServoVal(int degree) {
        double servoVal = degree/360;
        return servoVal;
    }



    /*******************************************************************************************
     * Method gyro360
     *  - Causes the Gyro to behave in 360 mode instead of 180 degree mode
     *******************************************************************************************/
    public double gyro360Radians(double targetAngle){
        double rotationalAngle = 0;

        if(targetAngle > Math.PI){
            rotationalAngle = targetAngle - (2 * Math.PI);
        }
        if(targetAngle < -Math.PI){
            rotationalAngle = targetAngle + (2 * Math.PI);
        }

        return rotationalAngle;
    }   // end method gyro360
}   // close the class
