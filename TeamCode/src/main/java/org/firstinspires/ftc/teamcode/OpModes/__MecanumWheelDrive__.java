package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private boolean armAngleDrop = false;
    private ElapsedTime dropTime = new ElapsedTime();
    private ElapsedTime buttonPressTime = new ElapsedTime();

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
        int armControl = 0;
        int armAngle = 0;
        double armAnglePower = 1;
        double intakeAngle = 0.5;

        //        double armUpDown;
        int armPosition = 0;
        int hangPosition = 0;
        dropTime.reset();
        buttonPressTime.reset();

        while (opModeIsActive()) {
            stickDrive = this.gamepad1.left_stick_y * DriveSpeed;
            turn = -this.gamepad1.right_stick_x * TurnSpeed;
            strafe = -this.gamepad1.left_stick_x * StrafeSpeed;

           drive.StrafeDrive(stickDrive, strafe, turn);


            if (gamepad1.a) {
                DriveSpeed = 1;
                StrafeSpeed = 1;
                TurnSpeed = 1;
            } else {
                DriveSpeed = 0.5;
                StrafeSpeed = 0.5;
                TurnSpeed = 0.5;
            }


//            if (gamepad1.a){
//                armAngle = robot.ARM_ANGLE_SCORE;
//            }

            if(gamepad2.left_bumper) {
                armAngle = armAngle + 6;
                if(armAngle > 0) armAngle = 0;
                armAnglePower = 1;
            } else if(gamepad2.right_bumper) {
                armAngle = armAngle - 6;
                armAnglePower = 0.25;
                if (armAngle < -1100) armAngle = -1100;
            }

            if(gamepad2.right_trigger > 0) {
                armControl = armControl + 20;
                if(armControl > 4900) armControl = 4900;
            } else if(gamepad2.left_trigger > 0){
                armControl = armControl - 20;
                if(armControl < 0 ) armControl = 0;
            }

            if(gamepad2.y){
                armAngleDrop = false;
                armAnglePower = 1;
                armAngle = -1100;
                armControl = 4900;
            } else if (gamepad2.x){
                armAnglePower = 0.25;
                armAngleDrop = true;
                dropTime.reset();
                armControl = 0;
            }

            if(armAngleDrop){
                if(dropTime.time() > 1.5){
                    armAngle = 0;
                    armAngleDrop = false;
                }
            }
            if(gamepad2.a) {
                // intake on
                robot.servoIntake.setPower(-1);
            } else if(gamepad2.b){
                // intake reverse
                robot.servoIntake.setPower(1);
            } else {
                // turn off the servo
                robot.servoIntake.setPower(0);
            }

            if(gamepad2.dpad_up && buttonPressTime.time() > 0.1) {
                buttonPressTime.reset();
                intakeAngle = intakeAngle + 0.03;
                if(intakeAngle > 1) intakeAngle = 1;
            } else if (gamepad2.dpad_down && buttonPressTime.time() > 0.1) {
                buttonPressTime.reset();
                intakeAngle = intakeAngle - 0.03;
                if(intakeAngle < 0) intakeAngle = 0;
            }

            // apply settings to motors and servos
            robot.servoIntakeAngle.setPosition(intakeAngle);
            robot.motorArmAngle.setPower(armAnglePower);
            robot.motorArmLength.setPower(1);
            robot.motorArmAngle.setTargetPosition(armAngle);
            robot.motorArmLength.setTargetPosition(armControl);

            telemetry.addData("armControl = ", armControl);
            telemetry.addData("armAngle = ", armAngle);
            telemetry.addData("servoIntakeAngle = ", intakeAngle);
            telemetry.addData("Status", "Running");
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();

        }
    }
}