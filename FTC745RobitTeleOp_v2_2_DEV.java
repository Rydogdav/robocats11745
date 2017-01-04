/**
Made by Daylan Davis and Collin Gustafson
Captained by Ryan Davitt
This program is the first TeleOp version for the FTC 11745 team of 2016-17 Slappy Robit 2.0.
This program (version 1.0) was originally used for Slappy 1.0 until the Scrimmage of '16, when the team made a total overhaul of Slappy 1.0.
Slappy 2.0 now resides in the Woodrow Wilson High School Robotics Room. Slappy 1.0 resides in soul.

@Verison 2.1.1

Version 2.1- RELEASE
Version 2.1.1- Tweaked gear numbers to acommadate for motor power curve
**/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_DEV.*;
import org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_DEV.DriveTeleOp;

import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_DEV.DriveTeleOp.motorBLeftv;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_DEV.DriveTeleOp.motorBRightv;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_DEV.DriveTeleOp.motorFLeftv;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_DEV.DriveTeleOp.motorFRightv;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_DEV.Shooting.ParticleShoot;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_DEV.DriveTeleOp.FieldCentricMecanum;



@TeleOp(name="TeleOp v2.2 DEV", group="TeleOp")  // @Autonomous(...) is the other common choice
public abstract class FTC745RobitTeleOp_v2_2_DEV extends LinearOpMode {
    /* Declare OpMode members. */
    public ElapsedTime runtime = new ElapsedTime();

    public static DcMotor motorFLeft = null;
    public static DcMotor motorFRight = null;
    public static DcMotor motorBLeft = null;
    public static DcMotor motorBRight = null;
    public static DcMotor motorLshoot = null;
    public static DcMotor motorRshoot = null;
    public static DcMotor motorThrasher = null;
    public static DcMotor motorLoadAmmo = null;
    public static Servo servoThrasherArm = null;
    public static Servo servoShooterPipe = null;
    public static GyroSensor gyroMain = null;
    public ColorSensor colorsensMain = null;
    public OpticalDistanceSensor colorsensLine = null;
    public OpticalDistanceSensor distanceMain = null;

    public static double lshootPower = 0.15;
    public static double rshootPower = 0.18;
    public static int shootpipeMax = 200;
    public static int shootpipeMin = 120;

    public double North = 0;
    public double East = 0;

    public static int currentHeading = 0;

    public static double currentGear = 0;
    public static String gearStatus = null;

    public static double Forward = 0;
    public static double Strafe = 0;
    public static double TurnCW = 0;

    final static public double Kf = 0.8;   //Ether's Kf
    final static public double Ks = 1;   //Ether's Ks
    final static public double Kt = 1;   //Ether's Kt

    public static double maxMotorPower = 1.0;
    final static int WHEEL_DIAMETER = 4;     //Diameter of the wheel in inches
    final static double WHEEL_DIAMETER_MM = WHEEL_DIAMETER * (25.4);
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_MM;
    final static int ENCODER_CPR = 1120;     //Encoder Counts per Revolution
    final static double GEAR_RATIO = 1;      //Gear Ratio

    final static double ROBOT_TURN_CIRCLE_RADIUS = 7.625;
    final static double ROBOT_TURN_CURCUMFERENCE = ROBOT_TURN_CIRCLE_RADIUS * Math.PI * 25.4;

    final static int TILE = 610;

    public static boolean thrasherToggle;


    @Override
    public void runOpMode() {
        motorFLeft = hardwareMap.dcMotor.get("motorFLeft");
        motorFRight = hardwareMap.dcMotor.get("motorFRight");
        motorBLeft = hardwareMap.dcMotor.get("motorBLeft");
        motorBRight = hardwareMap.dcMotor.get("motorBRight");
        //motorLshoot = hardwareMap.dcMotor.get("motorLshoot");
        //motorRshoot = hardwareMap.dcMotor.get("motorRshoot");
        motorThrasher = hardwareMap.dcMotor.get("motorThrasher");
        motorFRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBRight.setDirection(DcMotorSimple.Direction.REVERSE);
        ///motorLshoot.setDirection(DcMotorSimple.Direction.REVERSE);


        gyroMain = hardwareMap.gyroSensor.get("gyroMain");

        //servoMain = hardwareMap.servo.get("servoMain");
        //colorsensMain = hardwareMap.colorSensor.get("colorsensMain");
        //colorsensLine = hardwareMap.opticalDistanceSensor.get("colorsensLine");
        //distanceMain = hardwareMap.opticalDistanceSensor.get("distanceMain");
        //colorsensLine.enableLed(true);

        //recalibrate gyro TAKE OUT WHEN AUTONOMOUS IS WORKING
        gyroMain.calibrate();
        telemetry.addData("Status", "Initialized. Welcome user. v2.3 Now with PRECISION!!! (and Debug Gyro)!");
        //servoShooterPipe.setPosition(120);
        waitForStart();
        do {
            telemetry.addData("Status: ", "Good luck! Running: " + runtime.toString());

            //Driver 1 Controls
            // get driver gear input
            if (gamepad1.right_bumper && gamepad1.x) {
                gearStatus = "Full Power Activated";
                currentGear = 1.0;
            }
            if (gamepad1.right_bumper && gamepad1.a) {
                currentGear = 0.30;
                gearStatus = "Shooter Mode Activated";
            }
            if (gamepad1.right_bumper && gamepad1.b) {
                currentGear = 0.20;
                gearStatus = "Precision Mode Activated";
            }
            telemetry.addData("Gear: ", gearStatus);
            telemetry.update();
            //Driver 2 Controls
            if (gamepad2.right_bumper && gamepad2.a && motorLshoot.getPower() == 0) {
                motorLshoot.setPower(lshootPower);
                motorRshoot.setPower(rshootPower);
            }
            if (gamepad2.y) {
                ParticleShoot();
                sleep(2000);
                servoShooterPipe.setPosition(shootpipeMin);
                idle();
            }
            if (gamepad2.right_bumper && gamepad2.a && motorLshoot.getPower() > 0) {
                motorLshoot.setPower(0);
                motorRshoot.setPower(0);
                idle();
            }

            //get driver joystick input
            North = +gamepad1.left_stick_y;   //away from driver on field
            East = -gamepad1.left_stick_x;   //right with respect to driver
            TurnCW = -gamepad1.right_stick_x; //clockwise
            FieldCentricMecanum(North, East, TurnCW);

            if (gamepad2.a && thrasherToggle == false) {
                thrasherToggle = true;
            }
            if (gamepad2.a && thrasherToggle) {
                thrasherToggle = false;
                idle();
            }
            if (thrasherToggle) {
                motorThrasher.setPower(gamepad2.right_trigger);
                idle();
            }
            idle();
            //send power settings to the motors
            motorFLeft.setPower(motorFLeftv);
            motorFRight.setPower(motorFRightv);
            motorBLeft.setPower(motorBLeftv);
            motorBRight.setPower(motorBRightv);
            telemetry.addData("Gyro Reading", +gyroMain.getHeading());
            telemetry.update();
            idle();
        }while(opModeIsActive());
        idle();
    }

    }

