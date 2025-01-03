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

import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;


@TeleOp(name="Broken Bot", group="Test Mode")

public class BrokenBot extends LinearOpMode {

    // Declare OpMode members.
    private final static HWProfile robot = new HWProfile();
    private final LinearOpMode opMode = this;
    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);

    private final ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {
        robot.init(hardwareMap, true);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        /**leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");**/

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        /**leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);**/

        int armAnglePosition = 0;
        double strafe;
        double tempPosition = 0.3;
        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();
        robot.motorArmAngle.setPower(1);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.right_stick_y < 0) {
                armAnglePosition = armAnglePosition + 10;

                robot.motorRF.setPower(1);
                robot.motorLF.setPower(1);
                robot.motorRR.setPower(1);
                robot.motorLR.setPower(1);
            } else if(gamepad1.right_stick_y > 0) {
                armAnglePosition = armAnglePosition - 10;
                robot.motorRF.setPower(1);
                robot.motorLF.setPower(1);
                robot.motorRR.setPower(1);
                robot.motorLR.setPower(1);
            } else if(gamepad1.a) {
                robot.motorRF.setPower(0);
                robot.motorLF.setPower(0);
                robot.motorRR.setPower(0);
                robot.motorLR.setPower(0);
            }

            robot.motorArmAngle.setTargetPosition(armAnglePosition);


            if (gamepad1.dpad_up) {
                robot.motorLF.setPower(1);
                telemetry.addData("Motor LF ", robot.motorLF.getCurrentPosition());
            } else robot.motorLF.setPower(0);

            if (gamepad1.dpad_down) {
                robot.motorLR.setPower(1);
                telemetry.addData("Motor LR ", robot.motorLR.getCurrentPosition());
            } else robot.motorLR.setPower(0);

            if (gamepad1.dpad_right) {
                robot.motorRF.setPower(1);
                telemetry.addData("Motor RF ", robot.motorRF.getCurrentPosition());
            } else robot.motorRF.setPower(0);

            if (gamepad1.dpad_left) {
                robot.motorRR.setPower(1);
                telemetry.addData("Motor RR ", robot.motorRR.getCurrentPosition());
            } else robot.motorRR.setPower(0);


            telemetry.addData("Right Front Encoder = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("Right Rear Encoder = ", robot.motorRR.getCurrentPosition());
            telemetry.addData("Left Front Encoder = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("Left Rear Encoder = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
