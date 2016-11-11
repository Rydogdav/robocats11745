package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;


@Autonomous(name="Autonomous Main", group="Autonomous")
public class MainAutonomous extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorBRight;
    DcMotor motorBLeft;
    GyroSensor gyroMain;

    final static int ENCODER_CPR = 1440;     //Encoder Counts per Revolution
    final static double GEAR_RATIO = 1;      //Gear Ratio
    final static int WHEEL_DIAMETER = 4;     //Diameter of the wheel in inches
    final static int Correction = 31;


    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;



    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorBLeft = hardwareMap.dcMotor.get("motorBLeft");
        motorBRight = hardwareMap.dcMotor.get("motorBRight");
        gyroMain = hardwareMap.gyroSensor.get("gyroMain");

        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorBRight.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("CALIBRATING GYRO IN 3 SECONDS! ", "PLACE SLAPPY/SLOPPY IN PLACE!");
        sleep(3000);
        gyroMain.calibrate();
        telemetry.clearAll();
        telemetry.addData("INITILIZED:", "Autonomous V1 ACTIVATED");
    }
    private void ResetEncoder(){
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void LinearMove(int DISTANCE){
        final double ROTATIONS = DISTANCE / CIRCUMFERENCE;
        final double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;

        motorLeft.setTargetPosition((int) COUNTS); //Sets position in counts
        motorRight.setTargetPosition((int) COUNTS);
        motorBLeft.setTargetPosition((int) COUNTS);
        motorBRight.setTargetPosition((int) COUNTS);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to position
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setPower(0.5);
        motorRight.setPower(0.5);
        motorBLeft.setPower(0.5);
        motorBRight.setPower(0.5);
    }
    private void LinearTurnR(int DISTANCE){
        final double ROTATIONS = DISTANCE / CIRCUMFERENCE;
        final double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;

        motorLeft.setTargetPosition((int) COUNTS); //Sets position in counts
        motorRight.setTargetPosition((int) COUNTS);
        motorBLeft.setTargetPosition((int) COUNTS);
        motorBRight.setTargetPosition((int) COUNTS);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to position
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setPower(0.5); //Runs to position at this power
        motorRight.setPower(-0.5);
        motorBLeft.setPower(0.5);
        motorBRight.setPower(-0.5);

    }
    private void AllStop(){
        motorBLeft.setPower(0);
        motorBRight.setPower(0);
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
    @Override
    public void start() {
        LinearMove(72 - Correction);



    }

    @Override
    public void loop() {
        telemetry.addData("Left Position", motorLeft.getCurrentPosition());
        telemetry.addData("Right Position", motorRight.getCurrentPosition());
    }
}

