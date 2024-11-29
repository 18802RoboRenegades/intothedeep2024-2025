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

//@Autonomous(name = "RR", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
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
    public Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
    public Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
    public Pose2d midwayPose1 = new Pose2d(0,0,0);
    public Pose2d midwayPose1a = new Pose2d(0,0,0);
    public Pose2d intakeStack = new Pose2d(0,0,0);
    public Pose2d midwayPose2 = new Pose2d(0,0,0);
    public Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
    public Pose2d parkPose = new Pose2d(0,0, 0);


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, false);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        drive = new MecanumDrive(hardwareMap, initPose);



        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();

        mechOps.openClaw();
        mechOps.resetArm();

        // Wait for the DS start button to be touched.
        telemetry.addData("Selected Starting Position", startPosition);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {

            runAutonoumousMode(drive);
        }
    }   // end runOpMode()

    public void runAutonoumousMode(MecanumDrive thisDrive) {


        // make sure the bot has a good grip on the pixels
        /**
        mechOps.resetArm();
        mechOps.closeClaw();
        safeWaitSeconds(0.5);
        **/

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());

        //TODO : Code to drop Purple Pixel on Spike Mark
        /**
        mechOps.raiseArm(params.ARM_OUT);
        safeWaitSeconds(0.3);
         **/

        //Move robot to midwayPose1
        /***    Skipping this step for now
         Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());
         **/

        //For Blue Right and Red Left, intake pixel from stack
        if (startPosition == START_POSITION.BLUE_SPECIMEN ||
                startPosition == START_POSITION.RED_SAMPLE) {
            Actions.runBlocking(
                    thisDrive.actionBuilder(thisDrive.pose)
                            .strafeToLinearHeading(midwayPose1a.position, midwayPose1a.heading)
                            .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                            .build());

            //TODO : Code to intake pixel from stack
            safeWaitSeconds(0);

            //Move robot to midwayPose2 and to dropYellowPixelPose
            Actions.runBlocking(
                    thisDrive.actionBuilder(thisDrive.pose)
                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                            .build());
        }

        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(dropYellowPixelPose.position, dropYellowPixelPose.heading)
                        .build());


        //TODO : Code to drop Pixel on Backdrop;
        /***
        safeWaitSeconds(0.5);
        mechOps.liftLowJunction();
        safeWaitSeconds(0.5);
        mechOps.raiseArm(params.ARM_SCORE);
        safeWaitSeconds(1);
        mechOps.openClaw();
        safeWaitSeconds(0.4);
        mechOps.raiseArm(params.ARM_OUT);
        safeWaitSeconds(0.5);
        mechOps.liftReset();
         ***/


        //Move robot to park in Backstage
        Actions.runBlocking(
                thisDrive.actionBuilder(thisDrive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());

        /**
        mechOps.resetArm();
        mechOps.liftReset();
        safeWaitSeconds(1);
         **/
    }

    public void samples(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        dropPurplePixelPose = new Pose2d(25, 13, Math.toRadians(-45));
        dropYellowPixelPose = new Pose2d(23, 42, Math.toRadians(-90));
        moveBeyondTrussPose = new Pose2d(24, 15, Math.toRadians(-45));
        dropPurplePixelPose = new Pose2d(35, 4, Math.toRadians(0));
        dropYellowPixelPose = new Pose2d(25, 42,Math.toRadians(-90));
        moveBeyondTrussPose = new Pose2d(20, 4, Math.toRadians(0));
        dropPurplePixelPose = new Pose2d(28, -10, Math.toRadians(-45));
        dropYellowPixelPose = new Pose2d(31, 44, Math.toRadians(-90));
        moveBeyondTrussPose = new Pose2d(27, 3, Math.toRadians(-45));
        midwayPose1 = new Pose2d(30, 33, Math.toRadians(-90));
        parkPose = new Pose2d(3, 30, Math.toRadians(-90));

    }

    public void specimens(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        dropPurplePixelPose = new Pose2d(25, 13, Math.toRadians(-45));
        dropYellowPixelPose = new Pose2d(23, 42, Math.toRadians(-90));
        moveBeyondTrussPose = new Pose2d(24, 15, Math.toRadians(-45));
        dropPurplePixelPose = new Pose2d(35, 4, Math.toRadians(0));
        dropYellowPixelPose = new Pose2d(25, 42,Math.toRadians(-90));
        moveBeyondTrussPose = new Pose2d(20, 4, Math.toRadians(0));
        dropPurplePixelPose = new Pose2d(28, -10, Math.toRadians(-45));
        dropYellowPixelPose = new Pose2d(31, 44, Math.toRadians(-90));
        moveBeyondTrussPose = new Pose2d(27, 3, Math.toRadians(-45));
        midwayPose1 = new Pose2d(30, 33, Math.toRadians(-90));
        parkPose = new Pose2d(3, 30, Math.toRadians(-90));

        // Actions
        scoreSample(drive);

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

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initialized for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addData("    Blue Sample   ", "(X / ▢)");
            telemetry.addData("    Blue Specimen ", "(Y / Δ)");
            telemetry.addData("    Red Sample    ", "(B / O)");
            telemetry.addData("    Red Specimen  ", "(A / X)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_SAMPLE;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_SPECIMEN;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_SAMPLE;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_SPECIMEN;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }


}   // end class