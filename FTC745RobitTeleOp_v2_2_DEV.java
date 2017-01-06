package org.firstinspires.ftc.teamcode;

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

import static android.os.SystemClock.sleep;

@Autonomous(name="Auto v2.1 CLEAN DEV", group="Autonomous")
@Disabled

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

    public OpticalDistanceSensor distanceMain = null;
    final static double PERFECT_COLOR_VALUE = 0.3;

    public Servo servoRight;
    public Servo servoLeft;

    public double motorFLeftPower = 0;
    public double motorBLeftPower = 0;
    public double motorFRightPower = 0;
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
        telemetry.update();
        idle();
    }


    @Override
    public void runOpMode() throws InterruptedException {


    }

    private void LineFollower() {
        telemetry.addData("Color Value", distanceMain.getLightDetected());
        telemetry.update();
        while (true) {
            double correction = (PERFECT_COLOR_VALUE - distanceMain.getLightDetected());
            if (correction <= 0) {
                motorBLeftPower = 0.2 - correction;
                motorFLeftPower = 0.2 - correction;
                motorBRightPower = 0.2;
                motorFRightPower = 0.2;
            } else {
                motorBLeftPower = 0.2;
                motorFLeftPower = 0.2;
                motorBRightPower = 0.2 + correction;
                motorFRightPower = 0.2 + correction;
            }
            motorFLeft.setPower(motorFLeftPower);
            motorBLeft.setPower(motorBLeftPower);
            motorFRight.setPower(motorFRightPower);
            motorBRight.setPower(motorBRightPower);
        }
    }
}

