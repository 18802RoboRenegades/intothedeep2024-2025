package org.firstinspires.ftc.teamcode.Libs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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

    public void twistTurn90(){
        robot.servoTwist.setPosition(robot.INTAKE_TWIST_90);
    }

    public void twistTurnInit(){
        robot.servoTwist.setPosition(robot.INTAKE_TWIST_INIT);
    }

    public void setScoreSpecimen(){
        robot.motorArmAngle.setPower(1);
        robot.motorArmLength.setPower(1);
        robot.servoTwist.setPosition(robot.INTAKE_TWIST_INIT);
        robot.servoIntakeAngle.setPosition(robot.INTAKE_ANGLE_SCORE_SPECIMEN);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_PREP_SCORE_SPECIMEN);
        robot.motorArmLength.setTargetPosition(robot.ARM_LENGTH_SCORE_SPECIMEN);
    }

    public void scoreSpecimen(){
        robot.motorArmAngle.setPower(1);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_SCORE_SPECIMEN);
        robot.servoIntakeAngle.setPosition(robot.INTAKE_ANGLE_SCORE_SPECIMEN);
        opMode.sleep(500);
        robot.servoIntake.setPosition(robot.INTAKE_CLAW_OPEN);
    }

    public void setGrabSpecimen(){
        robot.motorArmLength.setPower(1);
        robot.motorArmLength.setTargetPosition(robot.ARM_LENGTH_RESET);
        robot.motorArmAngle.setPower(1);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_GRAB_SPECIMEN);
        robot.servoIntakeAngle.setPosition(robot.INTAKE_ANGLE_GRAB_SPECIMEN);
        robot.servoIntake.setPosition(robot.INTAKE_CLAW_OPEN);
    }

    public void removeSpecimen(){
        robot.motorArmAngle.setPower(1);
        robot.servoIntake.setPosition(robot.INTAKE_CLAW_CLOSE);
        opMode.sleep(300);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_REMOVE_SPECIMEN);
    }

    public void resetArm(){
//        robot.servoIntake.setPosition(robot.INTAKE_CLAW_OPEN);
        robot.servoIntakeAngle.setPosition(robot.INTAKE_ANGLE_GRAB_SPECIMEN);
        robot.motorArmAngle.setPower(0.4);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_RESET);
        robot.motorArmLength.setPower(1);
        robot.motorArmLength.setTargetPosition(robot.ARM_LENGTH_RESET);
    }

    public void grabSample(){
        robot.servoIntake.setPosition(robot.INTAKE_CLAW_OPEN);
        robot.motorArmAngle.setPower(0.3);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_GRAB_SAMPLE);
        robot.motorArmLength.setPower(1);
        robot.motorArmLength.setTargetPosition(robot.ARM_LENGTH_GRAB_SAMPLE);
        robot.servoIntakeAngle.setPosition(robot.INTAKE_ANGLE_GRAB_SAMPLE);
    }

    public void scoreSample(){
        robot.servoIntake.setPosition(robot.INTAKE_CLAW_OPEN);
        robot.motorArmAngle.setPower(1);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_SCORE_HIGH_BASKET);
        robot.motorArmLength.setPower(1);
        robot.motorArmLength.setTargetPosition(robot.ARM_LENGTH_SCORE_HIGH_BASKET);
    }

    public void climb(){
        robot.servoIntakeAngle.setPosition(robot.INTAKE_ANGLE_GRAB_SAMPLE);
        robot.motorArmAngle.setPower(1);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_GRAB_BAR);
        opMode.sleep(1500);
        robot.motorRR.setPower(1);
        robot.motorLR.setPower(1);
        robot.motorLF.setPower(1);
        robot.motorRF.setPower(1);
        opMode.sleep(500);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_CLIMB);
        opMode.sleep(1500);
        halt();
    }

    public void halt(){
        robot.motorRR.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
    }

    public void tensionRetractionString(){
        boolean notRetracted = true;
        int retractionPosition = 0;

        robot.motorArmLength.setPower(1);
        robot.motorArmLength.setTargetPosition(0);
        while(notRetracted){
            retractionPosition = retractionPosition-1;
            robot.motorArmLength.setTargetPosition(retractionPosition);
            if(robot.motorArmLength.getCurrent(CurrentUnit.AMPS) > 1){
                notRetracted = false;
                robot.motorArmLength.setPower(0);
                robot.motorArmLength.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.motorArmLength.setTargetPosition(0);
                robot.motorArmLength.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.motorArmLength.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            opMode.telemetry.addData(">", "##################################<");
            opMode.telemetry.addData(">", "                                  <");
            opMode.telemetry.addData(">", "PREPARING HARDWARE - DO NOT START <");
            opMode.telemetry.addData(">", "                                  <");
            opMode.telemetry.addData(">", "##################################<");
            opMode.telemetry.addData("Target Position = ", retractionPosition);
            opMode.telemetry.addData("motorArmLength Position = ", robot.motorArmLength.getCurrentPosition());
            opMode.telemetry.addData("motorArmLength Current Draw = ", robot.motorArmLength.getCurrent(CurrentUnit.AMPS));

            opMode.telemetry.update();
        }
    }

}   // close the RRMechOps class