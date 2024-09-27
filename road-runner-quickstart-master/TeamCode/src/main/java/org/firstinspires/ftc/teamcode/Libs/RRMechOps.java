package org.firstinspires.ftc.teamcode.Libs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;

public class RRMechOps{

    /**

        This is a Library used for

    **/

    public RRHWProfile robot;
    public LinearOpMode opMode;
    public CSAutoParams params;
    private HWProfile2 robot2 = new HWProfile2();

    private DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot2, opMode);

    /*
     * Constructor method
     */
    public RRMechOps(RRHWProfile myRobot, LinearOpMode myOpMode, CSAutoParams autoParams){
        robot = myRobot;
        opMode = myOpMode;
        params = autoParams;

    }   // close DriveMecanum constructor Method

    /*
     * ###########################################################################################
     * ###########################################################################################
     * #######################      SYSTEM CONTROLS      #########################################
     * ###########################################################################################
     * ###########################################################################################
     */



}   // close the RRMechOps class