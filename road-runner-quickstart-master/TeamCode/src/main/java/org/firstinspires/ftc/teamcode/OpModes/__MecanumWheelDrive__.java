package org.firstinspires.ftc.teamcode.OpModes;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;


 /**
 #########################################

                                CONTROLS MAPPING -- TBD

 #########################################

  ==GAMEPAD 1==

    --DRIVE--
    Left Stick -- Forward/Backward (y-axis) & Strafe Left/Right (x-axis)
    Right Stick -- Turn Left/Right (x-axis)

    --SPECIAL--
    Hold Left Bumper -- Drive Dampener (Slows down drive speed for needed precision movements.)

  ==GAMEPAD 2==

    --TO BE DETERMINED--

 **/
@TeleOp(name="Teleop - Tinkerfest", group="LinearOpMode")

 /**

 This is the DriveOpMode. This is the OpMode that is used for the driver-controlled portion, and
 is also sometimes used for testing.

 **/

public class __MecanumWheelDrive__ extends LinearOpMode
{

    private final static HWProfile2 robot = new HWProfile2();
    private final LinearOpMode opMode = this;
    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
    private DistanceSensor sensorColorRange;
    private Servo servoTest;
    private final boolean pad2input = true;

    private double DriveSpeed = 1;
    private double TurnSpeed = 1;
    private double StrafeSpeed = 1;

    private boolean IsOverrideActivated = false;

    public void runOpMode()
    {
        robot.init(hardwareMap);
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//        drive.haltandresetencoders();
        //runtime.reset();

        // run until the end of the match (driver presses STOP)
        double stickDrive = 0;
        double turn = 0;
        double strafe = 0;
        double leftPower = 0;
        double rightPower = 0;
//        double armUpDown;
        int armPosition = 0;
        int hangPosition = 0;

        while (opModeIsActive()) {
            stickDrive = this.gamepad1.left_stick_y * DriveSpeed;
            turn = this.gamepad1.right_stick_x * TurnSpeed;
            strafe = this.gamepad1.left_stick_x * StrafeSpeed;

           drive.StrafeDrive(stickDrive, strafe, turn);


            if (gamepad1.left_bumper) {
                DriveSpeed = 1;
                StrafeSpeed = 1;
                TurnSpeed = 1;
            } else {
                DriveSpeed = 0.5;
                StrafeSpeed = 0.5;
                TurnSpeed = 0.5;
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();

        }
    }
}
