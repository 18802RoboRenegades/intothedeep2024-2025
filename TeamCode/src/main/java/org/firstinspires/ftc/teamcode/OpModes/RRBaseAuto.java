/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode.Libs.RRMechOps;

@Autonomous(name = "RR Auto", group = "Competition", preselectTeleOp = "__MecanumWheelDrive__")
public class RRBaseAuto extends LinearOpMode {

    public static String TEAM_NAME = "Robo Renegades";
    public static int TEAM_NUMBER = 18802;

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_SAMPLE,
        BLUE_SPECIMEN,
        RED_SAMPLE,
        RED_SPECIMEN
    }
    public static START_POSITION startPosition;

    public final static HWProfile robot = new HWProfile();
    public LinearOpMode opMode = this;
    public RRMechOps mechOps = new RRMechOps(robot, opMode);

    //Initialize Pose2d as desired
    public Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
    public Pose2d specimenScoringPrepPosition = new Pose2d(0,0,0);
    public Pose2d specimenScoringPosition = new Pose2d(0, 0, 0);
    public Pose2d midwayPose1 = new Pose2d(0,0,0);
    public Pose2d coloredSample1 = new Pose2d(0,0,0);
    public Pose2d coloredSample2 = new Pose2d(0,0,0);
    public Pose2d coloredSample3 = new Pose2d(0,0,0);
    public Pose2d midwayPose2 = new Pose2d(0,0,0);
    public Pose2d specimenPickupPosition = new Pose2d(0,0,0);
    public Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
    public Pose2d parkPose = new Pose2d(0,0, 0);


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, false);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        drive = new MecanumDrive(hardwareMap, initPose);

        telemetry.addData(">", "##################################<");
        telemetry.addData(">", "                                  <");
        telemetry.addData(">", "PREPARING HARDWARE - DO NOT START <");
        telemetry.addData(">", "                                  <");
        telemetry.addData(">", "##################################<");
        telemetry.update();

        mechOps.tensionRetractionString();

        mechOps.closeClaw();
        mechOps.resetArm();
        robot.servoIntakeAngle.setPosition(robot.INTAKE_ANGLE_INIT);
        robot.servoTwist.setPosition(robot.INTAKE_TWIST_INIT);

        // Wait for the DS start button to be touched.
        while(!opModeIsActive()){
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.addData("##############", "##################");
            telemetry.addData("Retraction Position = ", robot.motorArmLength.getCurrentPosition());
            telemetry.addData("Arm Angle Encoder Value = ", robot.motorArmAngle.getCurrentPosition());
            telemetry.update();
        }

        waitForStart();
        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {

            scoreSpecimen(drive);
            grabColoredSample1(drive);
            grabColoredSample2(drive);
            grabSpecimen2(drive);
//            runAutonoumousMode(drive);
        }
    }   // end runOpMode()

    public void runAutonoumousMode(MecanumDrive thisDrive) {


        // make sure the bot has a good grip on the pixels
        Pose2d specimenScoringPrepPosition = new Pose2d(36,8,0);
        Pose2d specimenScoringPosition = new Pose2d(22, 6, 0);
        Pose2d midwayPose1 = new Pose2d(10,-33,Math.toRadians(180));
        Pose2d midwayPose2 = new Pose2d(55,-33,Math.toRadians(180));
        Pose2d midwayPose3 = new Pose2d(10,6,Math.toRadians(20));
        Pose2d midwayPose4 = new Pose2d(23,-30,0);
        Pose2d coloredSample1 = new Pose2d(60,-35,Math.toRadians(180));
        Pose2d coloredSample2 = new Pose2d(60,-55,0);
        Pose2d coloredSample3 = new Pose2d(0,0,0);
        Pose2d observationZonePosition = new Pose2d(5,-30,Math.toRadians(180));
        Pose2d specimenPickupPrepPosition = new Pose2d(15,-35,Math.toRadians(180));
        Pose2d specimenPickupPosition = new Pose2d(10,-35,Math.toRadians(180));
        Pose2d specimenPickupPositionFinal = new Pose2d(2,-35,Math.toRadians(180));
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0,-55, 0);

        safeWaitSeconds(0.5);
        if(opModeIsActive()) mechOps.setScoreSpecimen();
        safeWaitSeconds(0.25);
        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(specimenScoringPrepPosition.position, specimenScoringPrepPosition.heading)
                        .build());

        if(opModeIsActive()) mechOps.scoreSpecimen();

        //Move robot to midwayPose1
        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        if(opModeIsActive()) mechOps.resetArm();

         Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                        .strafeToLinearHeading(coloredSample1.position, coloredSample1.heading)
                        .strafeToLinearHeading(observationZonePosition.position, observationZonePosition.heading)
//                        .strafeToLinearHeading(midwayPose3.position, midwayPose3.heading)
//                        .strafeToLinearHeading(coloredSample2.position, coloredSample2.heading)
//                        .strafeToLinearHeading(observationZonePosition.position, observationZonePosition.heading)
//                        .strafeToLinearHeading(midwayPose4.position, midwayPose4.heading)
                        .build());

        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(specimenPickupPrepPosition.position, specimenPickupPrepPosition.heading)
                        .strafeToLinearHeading(specimenPickupPosition.position, specimenPickupPosition.heading)
                        .build());

        safeWaitSeconds(0.5);
        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(specimenPickupPositionFinal.position, specimenPickupPositionFinal.heading)
                        .build());

        safeWaitSeconds(0.2);
        mechOps.closeClaw();
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_REMOVE_SPECIMEN);

        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(midwayPose3.position, midwayPose3.heading)
                        .build());

        if(opModeIsActive()) mechOps.setScoreSpecimen();
        safeWaitSeconds(1);

        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(specimenScoringPrepPosition.position, specimenScoringPrepPosition.heading)
                        .build());

        if(opModeIsActive()) mechOps.scoreSpecimen();

        //Move robot to midwayPose1
        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        if(opModeIsActive()) mechOps.resetArm();


        //Move robot to park in the Observation Zone
        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        .build());

        /**
        mechOps.resetArm();
        mechOps.liftReset();
        safeWaitSeconds(1);
         **/
    }

    public void scoreSpecimen(MecanumDrive thisDrive) {
        // make sure the bot has a good grip on the pixels
        Pose2d specimenScoringPrepPosition = new Pose2d(36,8,0);
        Pose2d midwayPose1 = new Pose2d(20,-8,Math.toRadians(0));

        safeWaitSeconds(0.5);
        if(opModeIsActive()) mechOps.setScoreSpecimen();
        safeWaitSeconds(0.25);

        //Approach submersible to latch the specimen
        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(specimenScoringPrepPosition.position, specimenScoringPrepPosition.heading)
                        .build());

        // Engage the specimen with the submersible
        if(opModeIsActive()) mechOps.scoreSpecimen();


        // back away from the submersible before resetting arms
        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        if(opModeIsActive()) mechOps.resetArm();
        if(opModeIsActive()) mechOps.openClaw();
//        safeWaitSeconds(1000);
    }

    public void grabColoredSample1(MecanumDrive thisDrive) {
        // make sure the bot has a good grip on the pixels
        Pose2d midwayPose1a = new Pose2d(20,-27,Math.toRadians(180));
        Pose2d midwayPose2 = new Pose2d(55,-27,Math.toRadians(180));
        Pose2d coloredSample1 = new Pose2d(55,-38,Math.toRadians(180));
        Pose2d observationZonePosition = new Pose2d(5,-30,Math.toRadians(180));

        if(opModeIsActive()) mechOps.removeSpecimen();

        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(midwayPose1a.position, midwayPose1a.heading)
                        .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                        .strafeToLinearHeading(coloredSample1.position, coloredSample1.heading)
                        .strafeToLinearHeading(observationZonePosition.position, observationZonePosition.heading)
                        .build());

        if (opModeIsActive()) mechOps.resetArm();
    }

    public void grabColoredSample2(MecanumDrive thisDrive) {
        // make sure the bot has a good grip on the pixels
        Pose2d midwayPose1 = new Pose2d(55,-35,Math.toRadians(180));
        Pose2d coloredSample2 = new Pose2d(55,-48,Math.toRadians(180));
        Pose2d observationZonePosition = new Pose2d(5,-30,Math.toRadians(180));

        if(opModeIsActive()) mechOps.removeSpecimen();

        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .strafeToLinearHeading(coloredSample2.position, coloredSample2.heading)
                        .strafeToLinearHeading(observationZonePosition.position, observationZonePosition.heading)
                        .build());

        if(opModeIsActive()) mechOps.resetArm();

    }

    public void grabSpecimen2(MecanumDrive thisDrive) {
        // make sure the bot has a good grip on the pixels
        Pose2d specimenPickupPrepPosition = new Pose2d(2,-35,Math.toRadians(180));
        Pose2d midwayPose1 = new Pose2d(20,10,Math.toRadians(0));
        Pose2d specimenScoringPrepPosition = new Pose2d(36,8,Math.toRadians(0));

        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(specimenPickupPrepPosition.position, specimenPickupPrepPosition.heading)
                        .build());

        if(opModeIsActive()) mechOps.closeClaw();
        safeWaitSeconds(0.400);

        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .strafeToLinearHeading(specimenScoringPrepPosition.position, specimenPickupPrepPosition.heading)
                        .build());
    }



    public void samples(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        specimenScoringPosition = new Pose2d(25, 13, Math.toRadians(-45));
        dropYellowPixelPose = new Pose2d(23, 42, Math.toRadians(-90));
        specimenScoringPrepPosition = new Pose2d(24, 15, Math.toRadians(-45));
        specimenScoringPosition = new Pose2d(35, 4, Math.toRadians(0));
        dropYellowPixelPose = new Pose2d(25, 42,Math.toRadians(-90));
        specimenScoringPrepPosition = new Pose2d(20, 4, Math.toRadians(0));
        specimenScoringPosition = new Pose2d(28, -10, Math.toRadians(-45));
        dropYellowPixelPose = new Pose2d(31, 44, Math.toRadians(-90));
        specimenScoringPrepPosition = new Pose2d(27, 3, Math.toRadians(-45));
        midwayPose1 = new Pose2d(30, 33, Math.toRadians(-90));
        parkPose = new Pose2d(3, 30, Math.toRadians(-90));

    }

    public void scoreSample(MecanumDrive thisDrive){
        //Move robot to park in Backstage

        mechOps.resetArm();
        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());

        mechOps.scoreSample();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }


}   // end class