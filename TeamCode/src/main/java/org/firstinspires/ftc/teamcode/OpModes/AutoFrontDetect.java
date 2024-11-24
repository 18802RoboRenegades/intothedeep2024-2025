package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

@Autonomous(name = "Auto Front Side", group = "Competition")
@Disabled
public class AutoFrontDetect extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private final LinearOpMode opMode = this;

    FtcDashboard dashboard;

    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
    int position = 3;

    public AutoFrontDetect(){

    }

    //    @Override
    public void runOpMode() {
//        State runState = State.HIGH_JUNCTION_1;
        State runState = State.DETECT;

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        robot.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) requestOpModeStop();   // user requested to abort setup

        while (opModeIsActive()) {
            switch(runState){
                case DETECT:



                    break;

                case RIGHT:



                    runState = State.HALT;
                    break;

                case CENTER:



                    runState = State.HALT;
                    break;

                case LEFT:



                    runState = State.HALT;
                    break;

                case PARK:



                    runState = State.HALT;
                    break;


                case HALT:

                    // shut down all motors
                    drive.motorsHalt();

                    requestOpModeStop();    // request stoppage of the program

                    break;
            }   // end of switch(state)
        }   // end of while(opModeIsActive)

        requestOpModeStop();

        telemetry.addData("Path", "Complete");
        telemetry.update();

    } // end of opmode

    /*
     * Enumerate the states of the machine
     */
    enum State {
        DETECT, LEFT, RIGHT, CENTER, PARK, HALT
    }   // end of enum State


    /**
     * Initialize the TensorFlow Object Detection engine.
     */

}       //End Linear Op Mode