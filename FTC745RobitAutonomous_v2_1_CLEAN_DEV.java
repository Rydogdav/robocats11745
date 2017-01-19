
package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveAuton.AllStop;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveAuton.Fwd;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_RELEASE.DriveAuton.Xcurr;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_RELEASE.DriveAuton.Ycurr;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveAuton.Fwd;

@Autonomous(name="Auto v2.1 CLEAN DEV", group="Autonomous")


public class FTC745RobitAutonomous_v2_1_CLEAN_DEV extends LinearOpMode {
    DcMotor motorFRight;
    DcMotor motorFLeft;
    DcMotor motorBRight;
    DcMotor motorBLeft;

    GyroSensor gyroMain;

    public ColorSensor colorsensFLeft = null;
    public ColorSensor colorsensBLeft = null;
    public ColorSensor colorsensFRight = null;
    public ColorSensor colorsensBRight = null;

    public OpticalDistanceSensor distanceMainF = null;
    public OpticalDistanceSensor distanceMainB = null;


    public double motorFLeftPower = 0;
    public double motorBLeftPower = 0;    public double motorFRightPower = 0;
    public double motorBRightPower = 0;

    final static int WHEEL_DIAMETER = 4;     //Diameter of the wheel in inches
    final static double WHEEL_DIAMETER_MM = WHEEL_DIAMETER * (25.4);
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_MM;
    final static int ENCODER_CPR = 1120;     //Encoder Counts per Revolution
    final static double GEAR_RATIO = 1;      //Gear Ratio

    final static double ROBOT_TURN_CIRCLE_RADIUS = 7.625;
    final static double ROBOT_TURN_CURCUMFERENCE = ROBOT_TURN_CIRCLE_RADIUS * Math.PI * 25.4;


    String Alliance;
    String startingPosition;
    boolean selectionConfirmed = false;



    private void getAutonomousParameters() {
    /*press X for blue, press B for red (press F to pay respects)
        right bumper for choosing the alliance, left bumper for starting position (A,B,C)*/
        telemetry.addData("Right Bumper & X --> BLUE; B-->RED", "LEFT BUMPER & A, B, Y --> STARTING POSITION (A, B, C)");
        telemetry.update();

        do {
            if (gamepad1.b && gamepad1.right_bumper ||
                    gamepad2.b && gamepad2.right_bumper) Alliance = "Red";

            if (gamepad1.x && gamepad1.right_bumper ||
                    gamepad2.x && gamepad2.right_bumper) Alliance = "Blue";

            if (gamepad1.a && gamepad1.left_bumper ||
                    gamepad2.a && gamepad2.left_bumper) startingPosition = "A";

            if (gamepad1.b && gamepad1.left_bumper ||
                    gamepad2.b && gamepad2.left_bumper) startingPosition = "B";

            if (gamepad1.y && gamepad1.left_bumper ||
                    gamepad2.y && gamepad2.left_bumper) startingPosition = "C";

            if ((gamepad1.right_bumper && gamepad1.left_bumper) ||
                    (gamepad2.right_bumper && gamepad2.left_bumper))
                selectionConfirmed = true;

            telemetry.addData("Alliance ", Alliance);
            telemetry.addData("Starting Position ", startingPosition);
            telemetry.update();
            idle();
        } while (!selectionConfirmed);

        telemetry.addData("Locked in", Alliance, startingPosition);
        telemetry.addData("Color Value", distanceMainF.getLightDetected());
        telemetry.update();
        idle();
    }

    public void coordinateSet() {
        if (Alliance == "Blue") {
            if (startingPosition == "A") {
                Xcurr = -838;
                Ycurr = -1561;
            }
            if (startingPosition == "B") {
                Xcurr = -229;
                Ycurr = -1561;
            }
            if (startingPosition == "C") {
                Xcurr = 381;
                Ycurr = -1561;
            }
        }
        if (Alliance == "Red") {
            if (startingPosition == "A") {
                Xcurr = 838;
                Ycurr = 1561;
            }
            if (startingPosition == "B") {
                Xcurr = 229;
                Ycurr = -1561;
            }
            if (startingPosition == "C") {
                Xcurr = -381;
                Ycurr = -1561;
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        motorFLeft = hardwareMap.dcMotor.get("motorFLeft");
        motorFRight = hardwareMap.dcMotor.get("motorFRight");
        motorBLeft = hardwareMap.dcMotor.get("motorBLeft");
        motorBRight = hardwareMap.dcMotor.get("motorBRight");
        motorFRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBRight.setDirection(DcMotorSimple.Direction.REVERSE);
        distanceMainF = hardwareMap.opticalDistanceSensor.get("distanceMainF");

        waitForStart();

        if (Alliance == "Blue") {
            if (startingPosition == "A");
                Fwd(-838,-1179,0,true);
                //Fwd(Shooting);
                Fwd(1409,0,0,true);

            if (startingPosition == "B");
                Fwd(-229,-456,0,true);
                //Fwd(Shooting);
                Fwd(1409,0,0,true);

            if (startingPosition == "C");
                Fwd(381,-951,0,true);
                //Fwd(Shooting);
                Fwd(1404,0,0,true);
        }
        if (Alliance == "Red")
            if (startingPosition == "A")
                Fwd(838,-1179,0,true);
                //Fwd(Shooting);
                Fwd(-1409,0,0,true);

            if (startingPosition == "B")
                Fwd(1179,838,0,true);
                //Fwd(Shooting);
                Fwd(0,-1409,0,true);

            if (startingPosition == "C")
                Fwd(229,-456,0,true);
                //Fwd(Shooting);
                Fwd(-1404,0,0,true);



        idle();
    }

    public void LineFollower() {
        final double PERFECT_COLOR_VALUE = .825;
        double correctionA;
        double correctionB;
        final double MOTOR_BASE_POWER = 0.075;
        boolean lineFound = false;
        boolean lineFoundB = false;
        telemetry.setMsTransmissionInterval(250);
            distanceMainF.enableLed(true);
            do {
                correctionA = (PERFECT_COLOR_VALUE - distanceMainF.getLightDetected());
                correctionB = (PERFECT_COLOR_VALUE - distanceMainB.getLightDetected());
                if (correctionA < 0) {
                    lineFound = true;
                }
                if (!lineFound){
                    motorFLeft.setPower(MOTOR_BASE_POWER);
                    motorBLeft.setPower(MOTOR_BASE_POWER);
                    motorFRight.setPower(MOTOR_BASE_POWER);
                    motorBRight.setPower(MOTOR_BASE_POWER);
                }
                if(lineFound) {
                    motorFLeftPower = -.1;
                    motorBLeftPower = -.1;
                    motorFRightPower = .1;
                    motorBRightPower = .1;
                }
                if(correctionB < 0){
                    lineFoundB = true;
                }
                if(lineFoundB){
                    AllStop();
                    SystemClock.sleep(500);
                }
                if(correctionA < 0)
                    lineFound = true;
                motorFLeft.setPower(motorFLeftPower);
                motorBLeft.setPower(motorBLeftPower);
                motorFRight.setPower(motorFRightPower);
                motorBRight.setPower(motorBRightPower);
                idle();
                telemetry.addData("Color Value", distanceMainF.getLightDetected());
                telemetry.addData("Correction", correctionA);
                telemetry.update();
            } while(!isStopRequested());
        }
    }
