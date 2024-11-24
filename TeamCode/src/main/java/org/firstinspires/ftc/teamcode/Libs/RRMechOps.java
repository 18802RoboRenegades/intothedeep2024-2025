package org.firstinspires.ftc.teamcode.Libs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class RRMechOps{

    /**

        This is a Library used for

    **/

    public HWProfile robot;
    public LinearOpMode opMode;
    private HWProfile robot2 = new HWProfile();

    private DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot2, opMode);

    /*
     * Constructor method
     */
    public RRMechOps(HWProfile myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }   // close DriveMecanum constructor Method

    /*
     * ###########################################################################################
     * ###########################################################################################
     * #######################      SYSTEM CONTROLS      #########################################
     * ###########################################################################################
     * ###########################################################################################
     */



}   // close the RRMechOps class