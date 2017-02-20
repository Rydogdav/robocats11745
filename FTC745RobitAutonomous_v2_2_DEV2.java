package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.*;

import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.DriveAuton.ASSMove;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.DriveAuton.ASSMoveLF;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.DriveAuton.ResetEncoder;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.DriveAuton.AllStop;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.DriveAuton.Fwd;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.DriveAuton.HeadingTurn;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.DriveAuton.LinearMove;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.DriveAuton.MecanumAutonRight;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.DriveAuton.Xcurr;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.DriveAuton.Ycurr;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.Shooting.ParticleShootAuton;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.Shooting.ParticleShootAuton2;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.Forward;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.lshootPower;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorBLeft;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorBRight;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorFLeft;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorFRight;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorLshoot;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorRshoot;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.servoShooterGate;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.servoShooterPipe;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.motorBLeftv;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.motorBRightv;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.motorFLeftv;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.motorFRightv;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.DriveAuton.Thetacurr;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_2_DEV.DriveAuton.HEADING_TARGET;
//1C2 Imports
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

@Autonomous(name="Auto v2.2 DEV2", group="Autonomous")


public class FTC745RobitAutonomous_v2_2_DEV2 extends LinearOpMode {

    public ColorSensor colorsensFLeft = null;
    public ColorSensor colorsensBLeft = null;
    public ColorSensor colorsensFRight = null;
    public ColorSensor colorsensBRight = null;


    public static OpticalDistanceSensor distanceMainF = null;
    public OpticalDistanceSensor distanceMainB = null;
    public TouchSensor touchsensorF = null;
    public static GyroSensor gyroMainAuto = null;

    boolean PerfectRedFound = true;
    boolean test = false;

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

    public static boolean lfMove = false;



    private void getAutonomousParameters() {
    /*press X for blue, press B for red (press F to pay respects)
        right bumper for choosing the alliance, left bumper for starting position (A,B,C)*/
        telemetry.addData("Right Bumper & X --> BLUE; B--> RED", "LEFT BUMPER & A, B, Y --> STARTING POSITION (A, B, C)");
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
            if (gamepad1.b || gamepad2.b){
                gearInversion = false;
                robotName = "Sloppy";
            }
            if ((gamepad1.right_bumper && gamepad1.left_bumper) ||
                    (gamepad2.right_bumper && gamepad2.left_bumper))
                selectionConfirmed = true;

            telemetry.addData("Alliance ", Alliance);
            telemetry.addData("Starting Position ", startingPosition);
            telemetry.addData("Robot", robotName);
            telemetry.addLine("Press both bumpers when done!");
            if(devMode) telemetry.addLine("DevMode Activated");
            telemetry.update();
            idle();
        } while (!selectionConfirmed || isStopRequested());
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
        telemetry.addLine("DEBUG1");
        telemetry.update();
        if (robotName == "Slappy") {
            motorLshoot = hardwareMap.dcMotor.get("motorLshoot");
            motorRshoot = hardwareMap.dcMotor.get("motorRshoot");
            servoShooterPipe = hardwareMap.servo.get("servoShooterPipe");
            servoShooterGate = hardwareMap.servo.get("servoShooterGate");
            servoShooterGate.setDirection(Servo.Direction.REVERSE);
            motorLshoot.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        telemetry.addLine("DEBUG2");
        telemetry.update();
        gyroMainAuto = hardwareMap.gyroSensor.get("gyroMain");
        motorFRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBRight.setDirection(DcMotorSimple.Direction.REVERSE);
        distanceMainF = hardwareMap.opticalDistanceSensor.get("distanceMainF");
        distanceMainB = hardwareMap.opticalDistanceSensor.get("distanceMainB");
        telemetry.addLine("DEBUG3");
        telemetry.update();
        colorsensFLeft = hardwareMap.colorSensor.get("colorsensFLeft");
        colorsensFRight = hardwareMap.colorSensor.get("colorsensFRight");
        touchsensorF = hardwareMap.touchSensor.get("touchsensMain");

        telemetry.addLine("DEBUG4");
        telemetry.update();
        colorsensFRight.enableLed(true);
        SystemClock.sleep(3000);
        colorsensFRight.enableLed(false);
        /*coordinateSet();
        Xcurr = 0;
        Ycurr = 0;*/
        gyroMainAuto.calibrate();
        waitForStart();
        LineFinder1();
        /*while (isStopRequested()) {
            motorFLeft.setPower(0);
            motorBLeft.setPower(0);
            motorFRight.setPower(0);
            motorBRight.setPower(0);
            requestOpModeStop();
        }
        ASSMove(610, false, gearInversion);
        while (isStopRequested()) {
            motorFLeft.setPower(0);
            motorBLeft.setPower(0);
            motorFRight.setPower(0);
            motorBRight.setPower(0);
            requestOpModeStop();
        }
        if (robotName == "Slappy") HeadingTurn(225, gearInversion);

        if (robotName == "Sloppy") HeadingTurn(225, false);

        while (isStopRequested()){
            motorFLeft.setPower(0);
            motorBLeft.setPower(0);
            motorFRight.setPower(0);
            motorBRight.setPower(0);
            requestOpModeStop();
        }
        if(robotName == "Slappy") ParticleShootAuton();
        while (isStopRequested()){
            motorFLeft.setPower(0);
            motorBLeft.setPower(0);
            motorFRight.setPower(0);
            motorBRight.setPower(0);
            requestOpModeStop();
        }
       SystemClock.sleep(1000);
        if(robotName == "Slappy") ParticleShootAuton2();
        while (isStopRequested()){
            motorFLeft.setPower(0);
            motorBLeft.setPower(0);
            motorFRight.setPower(0);
            motorBRight.setPower(0);
            requestOpModeStop();
        }
        SystemClock.sleep(3000);
       ASSMove(1725, false, gearInversion);
        if (isStopRequested()){
            motorFLeft.setPower(0);
            motorBLeft.setPower(0);
            motorFRight.setPower(0);
            motorBRight.setPower(0);
            requestOpModeStop();
        }
        SystemClock.sleep(3000);*/
        idle();
    }
    /*public void AutonInstructions(){ //Why is this never used?
        if (Alliance == "Blue" && devMode == false) {
            if (startingPosition == "A") {
                Fwd(-838, -1179, 0, true, gearInversion);
                SystemClock.sleep(100);
                Fwd(60, -670, 0 ,true, gearInversion);
                if (robotName == "Slappy") ParticleShootAuton();
                SystemClock.sleep(100);
                Fwd(1409, 0, 0, true, gearInversion);
            }
            if (startingPosition == "B") {
                Fwd(-229, -456, 0, true, gearInversion);
                SystemClock.sleep(100);
                Fwd(60, -670, 0 ,true, gearInversion);
                if (robotName == "Slappy") ParticleShootAuton();
                SystemClock.sleep(100);
                Fwd(1409, 0, 0, true, gearInversion);
            }
            if (startingPosition == "C") {
                Fwd(381, -951, 0, true, gearInversion);
                SystemClock.sleep(100);
                Fwd(60, -670, 0 ,true, gearInversion);
                if (robotName == "Slappy") ParticleShootAuton();
                SystemClock.sleep(100);
                Fwd(1404, 0, 0, true, gearInversion);
            }
        }
        if (Alliance == "Red" && devMode == false) {
            if (startingPosition == "A") {
                Fwd(838, -1179, 0, true, gearInversion);
                SystemClock.sleep(100);
                Fwd(-60, -670, 0 ,true, gearInversion);
                if (robotName == "Slappy") ParticleShootAuton();
                SystemClock.sleep(100);
                Fwd(-1409, 0, 0, true, gearInversion);
            }
            if (startingPosition == "B") {
                Fwd(1179, 838, 0, true, gearInversion);
                SystemClock.sleep(100);
                Fwd(-60, -670, 0 ,true, gearInversion);
                if (robotName == "Slappy") ParticleShootAuton();
                SystemClock.sleep(100);
                Fwd(0, -1409, 0, true, gearInversion);
            }
            if (startingPosition == "C") {
                Fwd(229, -456, 0, true, gearInversion);
                SystemClock.sleep(100);
                Fwd(-60, -670, 0 ,true, gearInversion);
                if (robotName == "Slappy") ParticleShootAuton();
                SystemClock.sleep(100);
                Fwd(-1404, 0, 0, true, gearInversion);
            }
        }
    }*/
    public void LineFinder1() {
        /*ASSMoveLF(false, gearInversion);
        SystemClock.sleep(500);
        telemetry.addLine("Mecanum Activate");*/
        MecanumAutonRight(3000);
        telemetry.addLine("Mecanum Done");
        telemetry.update();
    }
    public void BeaconAuton() {
        colorsensFLeft.enableLed(false);
        colorsensFRight.enableLed(false);
        double csRedR;
        double csBlueR;
        int runLeft = 0;
        int runRight = 0;
        boolean moveComplete = false;
        do {
            boolean vote = false;
            final double PerfectRedFound = 3;
            final double PerfectBlueFound = 3;
            csRedR = colorsensFRight.red();
            csBlueR = colorsensFRight.blue();

            if (Alliance == "Red") {
                telemetry.addData("Right Red Value", csRedR);
                if (csRedR == PerfectRedFound - 1 || csRedR == PerfectRedFound + 1 || csRedR == PerfectRedFound) {
                    ASSMove(110, true, gearInversion);
                    moveComplete = true;
                } else {
                    ASSMove(110,false, gearInversion);
                    moveComplete= true;
                }
            }
            if (Alliance == "Blue") {
                telemetry.addData("Right Blue Value", csBlueR);
                if (csBlueR == PerfectBlueFound - 1|| csBlueR == PerfectBlueFound + 1 || csBlueR == PerfectBlueFound) {
                    ASSMove(110, true, gearInversion);
                } else {
                    ASSMove(110, false, gearInversion);
                }
            }

            telemetry.update();
        } while (isStopRequested() || !moveComplete);
        telemetry.addLine("Out of loop"); //If touch sensor is pressed, then it has either hit a beacon or the wall is '!touchsensorF.ispressed()' meant?
        telemetry.update();
    }
}