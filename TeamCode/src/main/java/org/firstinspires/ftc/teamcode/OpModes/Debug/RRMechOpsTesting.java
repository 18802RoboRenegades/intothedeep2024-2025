/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.OpModes.Debug;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;


@TeleOp(name="RRMechOps Testing", group="Test Mode")

public class RRMechOpsTesting extends LinearOpMode {

    // Declare OpMode members.
    private final static HWProfile robot = new HWProfile();
    private final LinearOpMode opMode = this;
    private RRMechOps mechOps;

    private final ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {
        robot.init(hardwareMap, true);
        mechOps = new RRMechOps(robot, opMode);

        int armAnglePosition = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        robot.motorArmAngle.setPower(1);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.a) mechOps.openClaw();
            if(gamepad1.b) mechOps.closeClaw();
            if(gamepad1.dpad_up) {
                mechOps.setScoreSpecimen();
                armAnglePosition = robot.motorArmAngle.getTargetPosition();
            }
            if(gamepad1.dpad_right){
                mechOps.scoreSpecimen();
                armAnglePosition = robot.motorArmAngle.getTargetPosition();
            }
            if(gamepad1.dpad_down){
                mechOps.setGrabSpecimen();
                armAnglePosition = robot.motorArmAngle.getTargetPosition();
            }

            if(gamepad1.right_trigger > 0){
                armAnglePosition = armAnglePosition + 10;
                robot.motorArmAngle.setTargetPosition(armAnglePosition);
            } else if(gamepad1.left_trigger > 0){
                armAnglePosition = armAnglePosition - 10;
                robot.motorArmAngle.setTargetPosition(armAnglePosition);
            }

            if(gamepad1.left_stick_button) climb();
            if(gamepad1.right_stick_button) halt();

            if(gamepad1.right_stick_y > 0){
                armAnglePosition = armAnglePosition - ((int) gamepad1.right_stick_y);
                if(armAnglePosition < 0) armAnglePosition =0;
                if(armAnglePosition > robot.ARM_ANGLE_SCORE_HIGH_BASKET) armAnglePosition = robot.ARM_LENGTH_SCORE_HIGH_BASKET;

                robot.motorArmAngle.setTargetPosition(armAnglePosition);
            } else if(gamepad1.right_stick_y < 0){
                armAnglePosition = armAnglePosition + ((int) gamepad1.right_stick_y);
                if(armAnglePosition <0) armAnglePosition =0;
                if(armAnglePosition > robot.ARM_ANGLE_SCORE_HIGH_BASKET) armAnglePosition = robot.ARM_LENGTH_SCORE_HIGH_BASKET;

                robot.motorArmAngle.setTargetPosition(armAnglePosition);
            }



            telemetry.addData("Open Claw = ", "A");
            telemetry.addData("Close Claw = ", "B");
            telemetry.addData("Set Score Specimen = ", "DPAD_UP");
            telemetry.addData("Score Specimen = ", "DPAD_DOWN");
            telemetry.addData("Grab Specimen = ", "DPAD_RIGHT");
            telemetry.addData("Arm Angle Position = ", armAnglePosition);
            telemetry.addData("Right Front Encoder = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("Right Rear Encoder = ", robot.motorRR.getCurrentPosition());
            telemetry.addData("Left Front Encoder = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("Left Rear Encoder = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void climb(){
        robot.servoIntakeAngle.setPosition(robot.INTAKE_ANGLE_GRAB_SAMPLE);
        robot.motorArmAngle.setPower(1);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_GRAB_BAR);
        sleep(1500);
        robot.motorRR.setPower(1);
        robot.motorLR.setPower(1);
        robot.motorLF.setPower(1);
        robot.motorRF.setPower(1);
        sleep(500);
        robot.motorArmAngle.setTargetPosition(robot.ARM_ANGLE_GRAB_SPECIMEN);
        sleep(1500);
        halt();
    }

    public void halt(){
        robot.motorRR.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
    }

}
