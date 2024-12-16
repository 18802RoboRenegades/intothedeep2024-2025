package org.firstinspires.ftc.teamcode.Libs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class RRMechOps{

    /**

        This is a Library used for

    **/

    public HWProfile robot;
    public LinearOpMode opMode;

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

    public void closeClaw(){
        robot.servoIntake.setPosition(robot.INTAKE_CLAW_CLOSE);
    }

    public void openClaw(){
        robot.servoIntake.setPosition(robot.INTAKE_CLAW_OPEN);
    }

    public void setScoreSpecimen(){
        robot.motorArmAngle.setPower(1);
        robot.motorArmLength.setPower(1);
        robot.servoTwist.setPosition(robot.INTAKE_TWIST_INIT);
        robot.servoIntakeAngle.setPosition(robot.INTAKE_ANGLE_SCORE_SPECIMEN);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_SCORE_SPECIMEN);
        robot.motorArmLength.setTargetPosition(robot.ARM_LENGTH_SCORE_SPECIMEN);
    }

    public void scoreSpecimen(){
        robot.motorArmAngle.setPower(1);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_SCORE_SPECIMEN + 300);
    }

    public void setGrabSpecimen(){
        robot.motorArmLength.setPower(1);
        robot.motorArmLength.setTargetPosition(robot.ARM_LENGTH_RESET);
        robot.motorArmAngle.setPower(1);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_GRAB_SPECIMEN);
        robot.servoIntakeAngle.setPosition(robot.INTAKE_ANGLE_GRAB_SPECIMEN);
        robot.servoIntake.setPosition(robot.INTAKE_CLAW_OPEN);
    }

    public void resetArm(){
        robot.servoIntake.setPosition(robot.INTAKE_CLAW_OPEN);
        robot.servoIntakeAngle.setPosition(robot.INTAKE_ANGLE_GRAB_SPECIMEN);
        robot.motorArmAngle.setPower(0.2);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_GRAB_SPECIMEN);
        robot.motorArmLength.setPower(1);
        robot.motorArmLength.setTargetPosition(robot.ARM_LENGTH_RESET);
    }

    public void scoreSample(){
        robot.servoIntake.setPosition(robot.INTAKE_CLAW_OPEN);
        robot.motorArmAngle.setPower(1);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_SCORE_HIGH_BASKET);
        robot.motorArmLength.setPower(1);
        robot.motorArmLength.setTargetPosition(robot.ARM_LENGTH_SCORE_HIGH_BASKET);

    }

}   // close the RRMechOps class