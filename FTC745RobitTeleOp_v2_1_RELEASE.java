
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="TeleOp v2.1.1 FULL", group="TeleOp")  // @Autonomous(...) is the other common choice
@Disabled

//@Disabled
public class FTC745RobitTeleOp_v2_1_RELEASE extends OpMode {
    /* Declare OpMode members. */
    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor motorFLeft = null;
    public DcMotor motorFRight = null;
    public DcMotor motorBLeft = null;
    public DcMotor motorBRight = null;
    public GyroSensor gyroMain = null;
    public Servo servoMain = null;
    public ColorSensor colorsensMain = null;
    public OpticalDistanceSensor colorsensLine = null;
    public OpticalDistanceSensor distanceMain = null;

    public double North = 0;
    public double East = 0;

    public int currentHeading = 0;

    public double Forward = 1;
    public double Strafe = 1;
    public double TurnCW = 1;

    public double currentGear = 0;
    public String gearStatus = null;
    final public double Kf = 1;   //Ether's Kf
    final public double Ks = 1;   //Ether's Ks
    final public double Kt = 0.8;   //Ether's Kt

    public double maxMotorPower = 1.0;


    //Code to run ONCE when the driver hits INIT

    @Override
    public void init() {

        motorFLeft = hardwareMap.dcMotor.get("motorFLeft");
        motorFRight = hardwareMap.dcMotor.get("motorFRight");
        motorBLeft = hardwareMap.dcMotor.get("motorBLeft");
        motorBRight = hardwareMap.dcMotor.get("motorBRight");
        motorFRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBRight.setDirection(DcMotorSimple.Direction.REVERSE);

        gyroMain = hardwareMap.gyroSensor.get("gyroMain");
        servoMain = hardwareMap.servo.get("servoMain");
        //colorsensMain = hardwareMap.colorSensor.get("colorsensMain");
        //colorsensLine = hardwareMap.opticalDistanceSensor.get("colorsensLine");
        //distanceMain = hardwareMap.opticalDistanceSensor.get("distanceMain");
        //colorsensLine.enableLed(true);

        //recalibrate gyro TAKE OUT WHEN AUTONOMOUS IS WORKING
        gyroMain.calibrate();
        telemetry.addData("Status", "Initialized. Welcome user. v2.3 Now with PRECISION!!! (and Debug Gyro)!");
        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //servoMain.setPosition(homeServoMain);
        //positionMain = servoMain.getPosition();
        // telemetry.addData("Status", "Initialized");
        servoMain.setPosition(0);
    }


    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    //Don't forget to run to phone!
    @Override
    public void loop() {
        telemetry.addData("Status: ", "Good luck! Running: " + runtime.toString());

        //get driver gear input
        if (gamepad1.right_bumper && gamepad1.a ) {
            currentGear = 1.0;
            gearStatus = "Full Power Activated";
        }
        if (gamepad1.right_bumper && gamepad1.b ) {
            currentGear = 0.30;
            gearStatus ="Shooter Mode Activated";
        }
        if (gamepad1.right_bumper && gamepad1.x ) {
            currentGear = 0.20;
            gearStatus = "Precision Mode Activated";
        }
        telemetry.addData("Gear: ",gearStatus);
        telemetry.update();

        //get driver joystick input
        North = + gamepad1.left_stick_y;   //away from driver on field
        East = - gamepad1.left_stick_x;   //right with respect to driver
        TurnCW =  - gamepad1.right_stick_x; //clockwise
        /*North = - gamepad1.left_stick_y;   //away from driver on field
        TurnCW =  + gamepad1.right_stick_x;*/ //clockwise

        //Convert from field-centric inputs to robot-centric commands
        //currentHeading = gyroMain.getHeading();
        currentHeading = 0;
        Forward = + North * Math.cos(currentHeading) + East * Math.sin(currentHeading);
        Strafe = - North * Math.sin(currentHeading) + East * Math.cos(currentHeading);

        //Scaling outputs for gear input and tuning constants
        Forward = currentGear * Kf * Forward;
        Strafe = currentGear * Ks * Strafe;
        TurnCW = currentGear * Kt * TurnCW;

        //apply inverse kinematics to the scaled input
        double motorFLeftv =  + Forward + TurnCW + Strafe;
        double motorFRightv = + Forward - TurnCW - Strafe;
        double motorBLeftv = + Forward + TurnCW - Strafe;
        double motorBRightv = + Forward - TurnCW + Strafe;

        //find maximum input
        double maxInput = Math.abs(motorFLeftv);
        if(Math.abs(motorFRightv) > maxInput) maxInput = Math.abs(motorFRightv);
        if(Math.abs(motorBLeftv) > maxInput) maxInput = Math.abs(motorBLeftv);
        if(Math.abs(motorBRightv) > maxInput) maxInput = Math.abs(motorBRightv);


        //normalize to maximum allowed motor power
        if(maxInput > maxMotorPower){
            motorFLeftv = maxMotorPower * motorFLeftv / maxInput;
            motorFRightv = maxMotorPower * motorFRightv / maxInput;
            motorBLeftv = maxMotorPower * motorBLeftv / maxInput;
            motorBRightv = maxMotorPower * motorBRightv / maxInput;
        }

        //send power settings to the motors
        motorFLeft.setPower(motorFLeftv);
        motorFRight.setPower(motorFRightv);
        motorBLeft.setPower(motorBLeftv);
        motorBRight.setPower(motorBRightv);
        telemetry.addData("Gyro Reading",+ gyroMain.getHeading());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
