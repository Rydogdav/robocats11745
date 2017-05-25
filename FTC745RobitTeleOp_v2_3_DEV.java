/**
 Made by Daylan Davis and Collin Gustafson
 Captained by Ryan Davitt
 This program is the first TeleOp version for the FTC 11745 team of 2016-17 Slappy Robit 2.0.
 This program (version 1.0) was originally used for Slappy 1.0 until the Scrimmage of '16, when the team made a total overhaul of Slappy 1.0.
 Slappy 2.0 now resides in the Woodrow Wilson High School Robotics Room. Slappy 1.0 resides in soul.
 @Verison 2.2
 Version 2.1- RELEASE
 Version 2.1.1- Tweaked gear numbers to acommadate for motor power curve
 Version 2.2- RELEASE
 Version 2.3- Post Qualifier Tournament code
 **/

package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveAuton.ResetEncoder;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveTeleOp.FieldCentricMecanum;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.motorBLeftv;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.motorBRightv;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.motorFLeftv;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.motorFRightv;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.Shooting.ParticleShootTele;

@TeleOp(name="TeleOp v2.3 DEV", group="TeleOp")
public class FTC745RobitTeleOp_v2_3_DEV extends LinearOpMode {
    /* Declare OpMode members. */
    public static DcMotor motorFLeft = null;
    public static DcMotor motorFRight = null;
    public static DcMotor motorBLeft = null;
    public static DcMotor motorBRight = null;
    public static DcMotor motorLshoot = null;
    public static DcMotor motorRshoot = null;
    public static DcMotor motorThrasher = null;
    public static Servo servoShooterPipe = null;
    public static Servo servoShooterGate = null;
    public static GyroSensor gyroMain = null;
    public ColorSensor colorsensMain = null;
    public OpticalDistanceSensor colorsensLine = null;
    public OpticalDistanceSensor distanceMain = null;
    public Servo servoMain = null;


    public static double lshootPower = 0.51;
    public static double rshootPower = 0.57;
    public static double shootpipeMax = 0.53;
    public static double shootpipeMin = 0.33;
    public static double shootgateMax = 0.78;
    public static double shootgateMin = 0.47;

    public double North = 0;
    public double East = 0;

    public static int currentHeading = 0;

    public static double currentGear = 0;
    public static String gearStatus = null;

    public static double Forward = 0;
    public static double Strafe = 0;
    public static double TurnCW = 0;

    final static public double Kf = 1;   
    final static public double Ks = 1;  
    final static public double Kt = 1;  

    public static double maxMotorPower = 1.0;
    final static int WHEEL_DIAMETER = 4;     
    final static double WHEEL_DIAMETER_MM = WHEEL_DIAMETER * (25.4);
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_MM;
    final static int ENCODER_CPR = 1120;    
    final static double GEAR_RATIO = 1;      

    final static double ROBOT_TURN_CIRCLE_RADIUS = 7.625;
    final static double ROBOT_TURN_CURCUMFERENCE = ROBOT_TURN_CIRCLE_RADIUS * Math.PI * 25.4;

    final static int TILE = 610; 

    @Override
    public void runOpMode() throws InterruptedException {
        boolean selectionConfirm = false;
        String robotName = "NO NAME! A = Slappy, B = Sloppy.";
        telemetry.addLine("Please State Robot Name:");
        telemetry.update();
        do {
            if (gamepad1.a || gamepad2.a) robotName = "Slappy";
            if (gamepad1.b || gamepad2.b) robotName = "Sloppy";
            telemetry.addData("Robot Name: ", robotName);
            telemetry.addLine("Press Y When Ready");
            telemetry.update();
            if (gamepad1.y || gamepad2.y) selectionConfirm = true;
        } while (selectionConfirm == false);
        telemetry.clear();
        motorFLeft = hardwareMap.dcMotor.get("motorFLeft");
        motorFRight = hardwareMap.dcMotor.get("motorFRight");
        motorBLeft = hardwareMap.dcMotor.get("motorBLeft");
        motorBRight = hardwareMap.dcMotor.get("motorBRight");
        if(robotName == "Slappy") {
            motorLshoot = hardwareMap.dcMotor.get("motorLshoot");
            motorRshoot = hardwareMap.dcMotor.get("motorRshoot");
            motorThrasher = hardwareMap.dcMotor.get("motorThrasher");
            servoShooterPipe = hardwareMap.servo.get("servoShooterPipe");
            servoShooterGate = hardwareMap.servo.get("servoShooterGate");
            motorLshoot.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        motorFLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBRight.setDirection(DcMotorSimple.Direction.REVERSE);

        gyroMain = hardwareMap.gyroSensor.get("gyroMain");

        //servoMain = hardwareMap.servo.get("servoMain");
        //colorsensMain = hardwareMap.colorSensor.get("colorsensMain");
        //colorsensLine = hardwareMap.opticalDistanceSensor.get("colorsensLine");
        //distanceMain = hardwareMap.opticalDistanceSensor.get("distanceMain");
        //colorsensLine.enableLed(true);

        gyroMain.calibrate();
        servoShooterPipe.setPosition(shootpipeMax);
        servoShooterGate.setPosition(shootgateMin);

        telemetry.addData("Status:", "Initialized. Welcome user. v2.3 DEV Active");
        telemetry.update();
        idle();
        waitForStart();
        telemetry.clearAll();
        do {

            if (gamepad1.right_bumper && gamepad1.x || gamepad2.right_bumper && gamepad2.x) {
                gearStatus = "Full Power Activated";
                currentGear = 0.9;
            }
            if (gamepad1.right_bumper && gamepad1.a || gamepad2.right_bumper && gamepad2.a) {
                currentGear = 0.30;
                gearStatus = "Shooter Mode Activated";
            }
            if (gamepad1.right_bumper && gamepad1.b || gamepad2.right_bumper && gamepad2.b) {
                currentGear = 0.20;
                gearStatus = "Precision Mode Activated";
            }
            telemetry.addData("Gear: ", gearStatus);
        
            if (gamepad1.left_bumper && gamepad1.a && motorLshoot.getPower() == 0 || gamepad2.left_bumper && gamepad2.a && motorLshoot.getPower() == 0) {
                SystemClock.sleep(150); 
                motorLshoot.setPower(lshootPower);
                motorRshoot.setPower(rshootPower);
                idle();
            }
            if (gamepad1.left_bumper && gamepad1.y || gamepad2.left_bumper && gamepad2.y) {
                SystemClock.sleep(150); 
                ParticleShootTele();
                idle();
            }
            if (gamepad1.left_bumper && gamepad1.a && motorLshoot.getPower() > 0 || gamepad2.left_bumper && gamepad2.a && motorLshoot.getPower() > 0) {
                SystemClock.sleep(150); 
                motorLshoot.setPower(0);
                motorRshoot.setPower(0);
                idle();
            }
            if (gamepad1.left_bumper && gamepad1.x || gamepad2.left_bumper && gamepad2.x){
                SystemClock.sleep(150); 
                servoShooterGate.setPosition(shootgateMax);
                SystemClock.sleep(750);
                servoShooterGate.setPosition(shootgateMin);
                idle();
            }
            if (gamepad1.left_bumper && gamepad1.b) {
                SystemClock.sleep(150);
                if (motorThrasher.getPower() != 0) {
                    motorThrasher.setPower(0);
                } else {
                    motorThrasher.setPower(.6);
                }
                idle();
            }
           
            if(robotName == "Slappy") {
                North = +gamepad1.left_stick_y;  
                East = -gamepad1.left_stick_x;  
                TurnCW = -gamepad1.right_stick_x;
                idle();
            }
            if(robotName == "Sloppy") {
                North = -gamepad1.left_stick_y;
                TurnCW = +gamepad1.right_stick_x;
                idle();
            }
            FieldCentricMecanum(North, East, TurnCW);

            idle();

            motorFLeft.setPower(motorFLeftv);
            motorFRight.setPower(motorFRightv);
            motorBLeft.setPower(motorBLeftv);
            motorBRight.setPower(motorBRightv);

            if(gamepad1.y || gamepad2.y){
                SystemClock.sleep(500);
                ResetEncoder();
                idle();
            }
            motorFLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Gyro Reading", +gyroMain.getHeading());
            telemetry.addData("Motor FLeft:", motorFLeft.getCurrentPosition());
            telemetry.addData("Motor BRight:", motorBRight.getCurrentPosition());
            telemetry.addData("Motor FRight:", motorFRight.getCurrentPosition());
            telemetry.addData("Motor BLeft:", motorBLeft.getCurrentPosition());
            telemetry.update();
            idle();
        } while(opModeIsActive() && !isStopRequested());
        idle();
        requestOpModeStop();
    }
}
