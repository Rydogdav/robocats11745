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

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.Shooting.ParticleShootTele;


@TeleOp(name="Shoot Test v1.0", group="TeleOp")  // @Autonomous(...) is the other common choice
public class FTC745RobitShootTestv1_0 extends LinearOpMode {
    /* Declare OpMode members. */
    public static DcMotor motorLshoot = null;
    public static DcMotor motorRshoot = null;
    public static Servo servoShooterPipe = null;
    public static Servo servoShooterGate = null;

    public static double lshootPower = 0.13;
    public static double rshootPower = 0.17;
    public static double shootpipeMax = 0.1;
    public static double shootpipeMin = 0.04;
    public static double shootgateMax = 0.27;
    public static double shootgateMin = 0.75;

    @Override
    public void runOpMode() {
        motorLshoot = hardwareMap.dcMotor.get("motorLshoot");
        motorRshoot = hardwareMap.dcMotor.get("motorRshoot");
        motorLshoot.setDirection(DcMotorSimple.Direction.REVERSE);
        servoShooterPipe = hardwareMap.servo.get("servoShooterPipe");

        telemetry.addData("Status", "Initialized. Welcome user. v2.3 Now with PRECISION!!! (and Debug Gyro)!");
        servoShooterPipe.setPosition(shootpipeMin);
        idle();
        waitForStart();
        //Driver 2 Controls
        while(opModeIsActive()){
            if (gamepad2.right_bumper && gamepad2.a && motorLshoot.getPower() == 0) {
                motorLshoot.setPower(lshootPower);
                motorRshoot.setPower(rshootPower);
                SystemClock.sleep(200);
                idle();
            }
            if (gamepad2.y) {
                if(motorLshoot.getPower() != lshootPower || motorRshoot.getPower() != rshootPower){
                    motorLshoot.setPower(lshootPower);
                    motorRshoot.setPower(rshootPower);
                    SystemClock.sleep(3000);
                }
                servoShooterPipe.setPosition(shootpipeMax);
                SystemClock.sleep(1000);
                servoShooterPipe.setPosition(shootpipeMin);
                SystemClock.sleep(500);
                servoShooterGate.setPosition(shootgateMax);
                SystemClock.sleep(1000);
                servoShooterGate.setPosition(shootgateMin);
                idle();
            }
            if (gamepad2.right_bumper && gamepad2.a && motorLshoot.getPower() > 0) {
                motorLshoot.setPower(0);
                motorRshoot.setPower(0);
                SystemClock.sleep(200);
                idle();
            }
            idle();
        }
        idle();
    }
}
