package org.firstinspires.ftc.teamcode;


import android.os.SystemClock;
import android.provider.Telephony;

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
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveAuton.ASSMove;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveAuton.AllStop;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveAuton.Fwd;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveAuton.HeadingTurn;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveAuton.LinearMove;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveAuton.MecanumAuton;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveAuton.Xcurr;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveAuton.Ycurr;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.DriveAuton.Fwd;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.Shooting.ParticleShootAuton;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_1_DEV.Shooting.ParticleShootAuton2;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.TurnCW;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorFLeft;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorBLeft;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorFRight;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorBRight;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorLshoot;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorRshoot;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.servoShooterPipe;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.servoShooterGate;

@Autonomous(name="Auto v2.2 CLEAN DEV", group="Autonomous")

public class FTC745RobitAutonomous_v2_2_DEV extends LinearOpMode {

    public ColorSensor colorsensFLeft = null;
    public ColorSensor colorsensBLeft = null;
    public ColorSensor colorsensFRight = null;
    public ColorSensor colorsensBRight = null;

    public OpticalDistanceSensor distanceMainF = null;
    public OpticalDistanceSensor distanceMainB = null;
    public static GyroSensor gyroMainAuto = null;

    public double motorFLeftPower = 0;
    public double motorBLeftPower = 0;
    public double motorFRightPower = 0;
    public double motorBRightPower = 0;

    public static boolean gearInversion = false;
    public static boolean devMode = false;
    public String robotName = "Null";

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
            if (gamepad1.a || gamepad2.a) {
                gearInversion = true;
                robotName = "Slappy";
            }
            if (gamepad1.b || gamepad2.b) {
                gearInversion = false;
                robotName = "Sloppy";
            }
            if ((gamepad1.right_bumper && gamepad1.left_bumper) ||
                    (gamepad2.right_bumper && gamepad2.left_bumper))
                selectionConfirmed = true;
            if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
                devMode = true;
            }

            telemetry.addData("Alliance ", Alliance);
            telemetry.addData("Starting Position ", startingPosition);
            telemetry.addData("Robot", robotName);
            telemetry.addLine("Press both bumpers when done!");
            if (devMode) telemetry.addLine("DevMode Activated");
            telemetry.update();
            idle();
        } while (!selectionConfirmed);
        telemetry.clearAll();
        telemetry.addData("Locked in", Alliance, startingPosition, robotName);
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
        getAutonomousParameters();
        motorFLeft = hardwareMap.dcMotor.get("motorFLeft");
        motorFRight = hardwareMap.dcMotor.get("motorFRight");
        motorBLeft = hardwareMap.dcMotor.get("motorBLeft");
        motorBRight = hardwareMap.dcMotor.get("motorBRight");
        if (robotName == "Slappy") {
            motorLshoot = hardwareMap.dcMotor.get("motorLshoot");
            motorRshoot = hardwareMap.dcMotor.get("motorRshoot");
            servoShooterPipe = hardwareMap.servo.get("servoShooterPipe");
            servoShooterGate = hardwareMap.servo.get("servoShooterGate");
            servoShooterGate.setDirection(Servo.Direction.REVERSE);
            motorLshoot.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        gyroMainAuto = hardwareMap.gyroSensor.get("gyroMain");
        motorFRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBRight.setDirection(DcMotorSimple.Direction.REVERSE);
        distanceMainF = hardwareMap.opticalDistanceSensor.get("distanceMainF");
        distanceMainB = hardwareMap.opticalDistanceSensor.get("distanceMainB");
        colorsensFLeft = hardwareMap.colorSensor.get("Color Sensor");
        gyroMainAuto.calibrate();
        //coordinateSet();
        waitForStart();
        /*Xcurr = 0;
        Ycurr = 0;*/
        while (isStopRequested()) {
            motorFLeft.setPower(0);
            motorBLeft.setPower(0);
            motorFRight.setPower(0);
            motorBRight.setPower(0);
            requestOpModeStop();
        }
        ASSMove(2000, false, gearInversion);
        while (isStopRequested()) {
            motorFLeft.setPower(0);
            motorBLeft.setPower(0);
            motorFRight.setPower(0);
            motorBRight.setPower(0);
            requestOpModeStop();
        }
        if (robotName == "Slappy") ParticleShootAuton();
        while (isStopRequested()) {
            motorFLeft.setPower(0);
            motorBLeft.setPower(0);
            motorFRight.setPower(0);
            motorBRight.setPower(0);
            requestOpModeStop();
        }
        SystemClock.sleep(1000);
        if (robotName == "Slappy") ParticleShootAuton2();
        while (isStopRequested()) {
            motorFLeft.setPower(0);
            motorBLeft.setPower(0);
            motorFRight.setPower(0);
            motorBRight.setPower(0);
            requestOpModeStop();
        }
        HeadingTurn(45, gearInversion);
        SystemClock.sleep(3000);
        ASSMove(2000, false, gearInversion);
        if (isStopRequested()) {
            motorFLeft.setPower(0);
            motorBLeft.setPower(0);
            motorFRight.setPower(0);
            motorBRight.setPower(0);
            requestOpModeStop();
        }
        SystemClock.sleep(3000);
        telemetry.addLine("Done");
        telemetry.update();
        idle();
    }

    public void AutonInstructions() {
        if (Alliance == "Blue" && devMode == false) {
            if (startingPosition == "A") {
                Fwd(-838, -1179, 0, true, gearInversion);
                SystemClock.sleep(100);
                Fwd(60, -670, 0, true, gearInversion);
                if (robotName == "Slappy") ParticleShootAuton();
                SystemClock.sleep(100);
                Fwd(1409, 0, 0, true, gearInversion);
            }
            if (startingPosition == "B") {
                Fwd(-229, -456, 0, true, gearInversion);
                SystemClock.sleep(100);
                Fwd(60, -670, 0, true, gearInversion);
                if (robotName == "Slappy") ParticleShootAuton();
                SystemClock.sleep(100);
                Fwd(1409, 0, 0, true, gearInversion);
            }
            if (startingPosition == "C") {
                Fwd(381, -951, 0, true, gearInversion);
                SystemClock.sleep(100);
                Fwd(60, -670, 0, true, gearInversion);
                if (robotName == "Slappy") ParticleShootAuton();
                SystemClock.sleep(100);
                Fwd(1404, 0, 0, true, gearInversion);
            }
        }
        if (Alliance == "Red" && devMode == false) {
            if (startingPosition == "A") {
                Fwd(838, -1179, 0, true, gearInversion);
                SystemClock.sleep(100);
                Fwd(-60, -670, 0, true, gearInversion);
                if (robotName == "Slappy") ParticleShootAuton();
                SystemClock.sleep(100);
                Fwd(-1409, 0, 0, true, gearInversion);
            }
            if (startingPosition == "B") {
                Fwd(1179, 838, 0, true, gearInversion);
                SystemClock.sleep(100);
                Fwd(-60, -670, 0, true, gearInversion);
                if (robotName == "Slappy") ParticleShootAuton();
                SystemClock.sleep(100);
                Fwd(0, -1409, 0, true, gearInversion);
            }
            if (startingPosition == "C") {
                Fwd(229, -456, 0, true, gearInversion);
                SystemClock.sleep(100);
                Fwd(-60, -670, 0, true, gearInversion);
                if (robotName == "Slappy") ParticleShootAuton();
                SystemClock.sleep(100);
                Fwd(-1404, 0, 0, true, gearInversion);
            }
            if (devMode) {
                telemetry.addLine("Stage 1");
                telemetry.update();
                HeadingTurn(180, gearInversion);
                SystemClock.sleep(100);
                HeadingTurn(0, gearInversion);
                SystemClock.sleep(10000);
                telemetry.addLine("Stage 1 Complete");
                telemetry.update();
                SystemClock.sleep(1000);
                telemetry.addLine("Stage 2");
                telemetry.update();
                ASSMove(2000, true, gearInversion);
                SystemClock.sleep(500);
                ASSMove(2000, false, gearInversion);
                telemetry.addLine("Stage 2 Complete");
                telemetry.update();
                SystemClock.sleep(1000);
                telemetry.addLine("Stage 3");
                telemetry.update();
                Fwd(381, -951, 0, true, gearInversion);
                SystemClock.sleep(500);
                Fwd(0, 0, 180, true, gearInversion);
                SystemClock.sleep(500);
                Fwd(381, -1561, 0, true, gearInversion);
                SystemClock.sleep(10000);
                telemetry.addLine("Stage 3 Complete");
                telemetry.addLine("devMode Run Complete!");
                telemetry.update();
            }
        }
    }

    public void LineFollower() {
        final double PERFECT_COLOR_VALUE = .825;
        double correctionA;
        double correctionB;
        final double MOTOR_BASE_POWER = 0.1;
        boolean lineFound = false;
        boolean lineFoundB = false;
        String codePosition = "Nowhere";
        telemetry.setMsTransmissionInterval(250);
        distanceMainF.enableLed(true);
        do {
            correctionA = (PERFECT_COLOR_VALUE - distanceMainF.getLightDetected());
            correctionB = (PERFECT_COLOR_VALUE - distanceMainB.getLightDetected());

            if (correctionA < 0) {
                lineFound = true;
                correctionA = (PERFECT_COLOR_VALUE - distanceMainF.getLightDetected());
                correctionB = (PERFECT_COLOR_VALUE - distanceMainB.getLightDetected());
                codePosition = "Line found";
            }

            if (!lineFound) {
                motorFLeft.setPower(MOTOR_BASE_POWER);
                motorBLeft.setPower(MOTOR_BASE_POWER);
                motorFRight.setPower(MOTOR_BASE_POWER);
                motorBRight.setPower(MOTOR_BASE_POWER);
                correctionA = (PERFECT_COLOR_VALUE - distanceMainF.getLightDetected());
                correctionB = (PERFECT_COLOR_VALUE - distanceMainB.getLightDetected());
                codePosition = "Rollin'";
            }
            if (lineFound) {
                motorFLeftPower = -.1;
                motorBLeftPower = -.1;
                motorFRightPower = .1;
                motorBRightPower = .1;
                correctionA = (PERFECT_COLOR_VALUE - distanceMainF.getLightDetected());
                correctionB = (PERFECT_COLOR_VALUE - distanceMainB.getLightDetected());
                codePosition = "Line found spin";
            }
            if (correctionB < 0) {
                lineFoundB = true;
                correctionA = (PERFECT_COLOR_VALUE - distanceMainF.getLightDetected());
                correctionB = (PERFECT_COLOR_VALUE - distanceMainB.getLightDetected());
            }
            if (lineFoundB) {
                AllStop();
                SystemClock.sleep(500);
                if (lineFoundB) {
                    motorFLeft.setPower(MOTOR_BASE_POWER - correctionA);
                    motorBLeft.setPower(MOTOR_BASE_POWER - correctionA);
                    motorFRight.setPower(MOTOR_BASE_POWER);
                    motorBRight.setPower(MOTOR_BASE_POWER);
                } else
                    motorFLeft.setPower(MOTOR_BASE_POWER);
                motorBLeft.setPower(MOTOR_BASE_POWER);
                motorFRight.setPower(MOTOR_BASE_POWER - correctionA);
                motorBRight.setPower(MOTOR_BASE_POWER - correctionA);
                codePosition = "Turning with Corresction";
            }
            idle();
            telemetry.addData("Color Value", distanceMainF.getLightDetected());
            telemetry.addData("CorrectionA", correctionA);
            telemetry.addData("CorrectionB", correctionB);
            telemetry.addLine(codePosition);
            telemetry.update();
        } while (!isStopRequested());
    }

    public void BeaconAuton() {
        final double PerfectRedFound = 10;
        final double PerfectBlueFound = 3;
        telemetry.addData("Red Value", colorsensFLeft.red());
        telemetry.addData("Blue Value", colorsensFLeft.blue());

        if (Alliance == "Red"){
            if (colorsensFLeft.red() == PerfectRedFound) {
                MecanumAuton();
            }
        }
        if (Alliance == "Blue") {
            if (colorsensFLeft.blue() == PerfectBlueFound) {
                MecanumAuton();
            }
        }

    }
}

