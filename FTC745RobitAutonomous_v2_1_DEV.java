package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="Auto v2.1 DEV", group="Autonomous")
@Disabled

public class FTC745RobitAutonomous_v2_1_DEV extends LinearOpMode {
    DcMotor motorFRight;
    DcMotor motorFLeft;
    DcMotor motorBRight;
    DcMotor motorBLeft;

    GyroSensor gyroMain;

    public ColorSensor colorsensFLeft = null;
    public ColorSensor colorsensBLeft = null;
    public ColorSensor colorsensFRight = null;
    public ColorSensor colorsensBRight = null;

    public OpticalDistanceSensor colorsensLine = null;
    public OpticalDistanceSensor distanceMain = null;

    public Servo servoRight;
    public Servo servoLeft;

    final static int WHEEL_DIAMETER = 4;     //Diameter of the wheel in inches
    final static double WHEEL_DIAMETER_MM = WHEEL_DIAMETER * (25.4);
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_MM;
    final static int ENCODER_CPR = 1120;     //Encoder Counts per Revolution
    final static double GEAR_RATIO = 1;      //Gear Ratio

    final static double ROBOT_TURN_CIRCLE_RADIUS = 7.625;
    final static double ROBOT_TURN_CURCUMFERENCE = ROBOT_TURN_CIRCLE_RADIUS * Math.PI * 25.4;

    final static double TILE = 610;  // 24" in millimeters

    double GYRO_HEADING = 0;
    java.lang.String name = "Auto v1.1";
    java.lang.String Alliance = "Not Selected";
    java.lang.String startingPosition = "Not Selected";

     private boolean selectionConfirmed = false;

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

        } while (!selectionConfirmed);

            telemetry.addData("Locked in", Alliance, startingPosition);
            telemetry.update();

    }
        //BLESS THIS MESS
        private void ResetEncoder(){
            motorFLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motorFLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorFLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        private void LinearMove(double DISTANCE, boolean FORWARDS){
            final double ROTATIONS = DISTANCE / CIRCUMFERENCE;
            final double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
            int DIRECTION_MULTIPLIER;

            ResetEncoder();
            if(FORWARDS == true){
                DIRECTION_MULTIPLIER = 1;
            } else {
                DIRECTION_MULTIPLIER = -1;
            }
            boolean move = true;
                do{
                    motorFLeft.setTargetPosition((int) COUNTS); //Sets position in counts
                    motorFRight.setTargetPosition((int) COUNTS);
                    motorBLeft.setTargetPosition((int) COUNTS);
                    motorBRight.setTargetPosition((int) COUNTS);

                    motorFLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to position
                    motorFRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorBLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorBRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    motorFLeft.setPower(DIRECTION_MULTIPLIER * 0.5);
                    motorFRight.setPower(DIRECTION_MULTIPLIER * 0.5);
                    motorBLeft.setPower(DIRECTION_MULTIPLIER * 0.5);
                    motorBRight.setPower(DIRECTION_MULTIPLIER * 0.5);
                    idle();
            } while(move);
            move = false;
            idle();
        }
        private void LinearMoveSlow(double DISTANCE, boolean FORWARDS){
            final double ROTATIONS = DISTANCE / CIRCUMFERENCE;
            final double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
            int DIRECTION_MULTIPLIER;

            ResetEncoder();
            if(FORWARDS == true){
                DIRECTION_MULTIPLIER = 1;
            } else {
                DIRECTION_MULTIPLIER = -1;
            }
            boolean move = true;
            do {
                motorFLeft.setTargetPosition((int) COUNTS); //Sets position in counts
                motorFRight.setTargetPosition((int) COUNTS);
                motorBLeft.setTargetPosition((int) COUNTS);
                motorBRight.setTargetPosition((int) COUNTS);

                motorFLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to position
                motorFRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorFLeft.setPower(DIRECTION_MULTIPLIER * 0.1);
                motorFRight.setPower(DIRECTION_MULTIPLIER * 0.1);
                motorBLeft.setPower(DIRECTION_MULTIPLIER * 0.1);
                motorBRight.setPower(DIRECTION_MULTIPLIER * 0.1);
                idle();
            } while(move);
            move = false;
            idle();
        }
        /*private void LinearMoveB(double DISTANCE){
            final double ROTATIONS = DISTANCE / CIRCUMFERENCE;
            final double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;

            ResetEncoder();
            motorFLeft.setTargetPosition((int) COUNTS + motorFLeft.getCurrentPosition()); //Sets position in counts
            motorFRight.setTargetPosition((int) COUNTS + motorFRight.getCurrentPosition());
            motorBLeft.setTargetPosition((int) COUNTS + motorBLeft.getCurrentPosition());
            motorBRight.setTargetPosition((int) COUNTS + motorBRight.getCurrentPosition());

            motorFLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to position
            motorFRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFLeft.setPower(-0.5);
            motorFRight.setPower(-0.5);
            motorBLeft.setPower(-0.5);
            motorBRight.setPower(-0.5);
        /
        private void LinearTurnR(double DISTANCE){
            final double ROTATIONS = DISTANCE / CIRCUMFERENCE;
            final double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;

            ResetEncoder();
            motorFLeft.setTargetPosition((int) COUNTS); //Sets position in counts
            motorFRight.setTargetPosition((int)-COUNTS);
            motorBLeft.setTargetPosition((int)COUNTS);
            motorBRight.setTargetPosition((int)-COUNTS);

            motorFLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to position
            motorFRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //Collin is the best!!!!!!!
            motorBLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFLeft.setPower(0.5); //Runs to position at this power
            motorFRight.setPower(-0.5);
            motorBLeft.setPower(0.5);
            motorBRight.setPower(-0.5);

        }
        private void LinearTurnL(double DISTANCE){
            final double ROTATIONS = DISTANCE / CIRCUMFERENCE;
            final double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;

            ResetEncoder();
            motorFLeft.setTargetPosition((int) COUNTS); //Sets position in counts
            motorFRight.setTargetPosition((int)-COUNTS);
            motorBLeft.setTargetPosition((int)COUNTS);
            motorBRight.setTargetPosition((int)-COUNTS);

            motorFLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to position
            motorFRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFLeft.setPower(-0.5); //Runs to position at this power
            motorFRight.setPower(0.5);
            motorBLeft.setPower(-0.5);
            motorBRight.setPower(0.5);

        }*/
        private void HeadingTurn(double HEADING){
            double POWER_RIGHT = 1;
            double POWER_LEFT = 1;
            boolean move = true;
            do{
                if (HEADING <= 180 && GYRO_HEADING >= 180) {
                    if (HEADING != GYRO_HEADING) {
                        POWER_LEFT = -0.2; //Runs to position at this power
                        POWER_RIGHT = 0.2;
                    }else{
                        AllStop();
                    }
                } else {
                    if (HEADING != GYRO_HEADING) {
                        POWER_LEFT = 0.2; //Runs to position at this power
                        POWER_RIGHT = -0.2;
                    } else {
                        AllStop();
                    }
                }
                idle();
            } while(move);
        }
        private void ASSMove(double DISTANCE, boolean FORWARDS){ //"AutomatedStabilitySystemMove"
            double HEADING_TARGET;
            if(GYRO_HEADING > 180){
                HEADING_TARGET = GYRO_HEADING - 180;
            } else {
                HEADING_TARGET = GYRO_HEADING + 180;
            }
            int DIRECTION_MULTIPLIER;
            double GYRO_HEADING_NEW;
            double POWER_RIGHT = 0.5;
            double POWER_LEFT = 0.5;
            final double ROTATIONS = DISTANCE / CIRCUMFERENCE;
            final double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
            LinearMove(DISTANCE, FORWARDS);
            if(FORWARDS){
                DIRECTION_MULTIPLIER = 1;
            } else {
                DIRECTION_MULTIPLIER = -1;
            }
            boolean move = true;
            motorFLeft.setTargetPosition((int) COUNTS); //Sets position in counts
            motorFRight.setTargetPosition((int) COUNTS);
            motorBLeft.setTargetPosition((int) COUNTS);
            motorBRight.setTargetPosition((int) COUNTS);

            motorFLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to position
            motorFRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            do {

                if(GYRO_HEADING > 180){
                    GYRO_HEADING_NEW = GYRO_HEADING - 180;
                } else {
                    GYRO_HEADING_NEW = GYRO_HEADING + 180;
                }
                if (GYRO_HEADING_NEW < HEADING_TARGET) {//If Statement logic is inverted due to the headings being set to the 90-270 side instead of the 270-90 side.
                    double HEADING_DELTA = (GYRO_HEADING_NEW - HEADING_TARGET) / 10;
                    POWER_RIGHT = POWER_RIGHT + HEADING_DELTA;
                    POWER_LEFT = POWER_LEFT - HEADING_DELTA;
                }
                if(GYRO_HEADING_NEW > HEADING_TARGET){
                    double HEADING_DELTA = (HEADING_TARGET - GYRO_HEADING_NEW) / 10;
                    POWER_LEFT = POWER_LEFT + HEADING_DELTA;
                    POWER_RIGHT = POWER_RIGHT - HEADING_DELTA;
                }
                if(GYRO_HEADING == HEADING_TARGET){
                    POWER_LEFT = 0.5;
                    POWER_RIGHT = 0.5;
                }
                motorFLeft.setPower(DIRECTION_MULTIPLIER * POWER_LEFT);
                motorFRight.setPower(DIRECTION_MULTIPLIER * POWER_RIGHT);
                motorBLeft.setPower(DIRECTION_MULTIPLIER * POWER_LEFT);
                motorBRight.setPower(DIRECTION_MULTIPLIER * POWER_RIGHT);
                GYRO_HEADING = gyroMain.getHeading();
                idle();
            } while(move);
            AllStop();
            idle();
        }
        private void LinearMoveTriangulateXTurn(double XDISTANCE, double YDISTANCE){
            double CDISTANCE = Math.hypot(XDISTANCE, YDISTANCE);
            double TURNANGLE = Math.tan(YDISTANCE / XDISTANCE);
            double TURNBACK = (180 - TURNANGLE);

            final double ROTATIONSC = CDISTANCE / CIRCUMFERENCE;
            CDISTANCE = ENCODER_CPR * ROTATIONSC * GEAR_RATIO;

            final double ROTATIONSTM = TURNANGLE / CIRCUMFERENCE;
            TURNANGLE = ENCODER_CPR * ROTATIONSTM * GEAR_RATIO;

            final double ROTATIONSTB = TURNBACK / CIRCUMFERENCE;
            TURNBACK = ENCODER_CPR * ROTATIONSTB * GEAR_RATIO;

            HeadingTurn(TURNANGLE);
            LinearMove(CDISTANCE, true);
            HeadingTurn(TURNBACK);
        }
        private void AllStop(){
            motorBLeft.setPower(0);
            motorBRight.setPower(0);
            motorFLeft.setPower(0);
            motorFRight.setPower(0);
        }
        private void LineFollower() {
            while(colorsensLine.getRawLightDetected() < 0.5){
                ASSMove(TILE * 4, true);
            } if (colorsensLine.getRawLightDetected() >= 0.5) {
                telemetry.addLine("LINE DETECTED");
                LinearMoveSlow(TILE * 0.5, true);
                BeaconPusher();
            }
        }
        /*private void BeaconPusher() {
           if (Alliance == "Red" && (colorsensFLeft.argb() >= 7)) {

            } else {
                servoRight.setPosition(90);
                LinearMoveSlow(TILE * 0.25, true);
                }
            if (Alliance == "Blue" && (colorsensFLeft.argb() <= 3)) {

            } else {
                servoRight.setPosition(90);
                LinearMoveSlow(TILE * 0.25, true);
                }
            }*/

        private void BeaconPusher() {
            //strafe 20 mm

        }
        private void BeaconVote(String leftVote, String rightVote) {

        }
        public void runOpMode() {
            telemetry.addData("Start ", name, " initialization...");
            telemetry.update();

            motorFLeft = hardwareMap.dcMotor.get("motorFLeft");
            motorFRight = hardwareMap.dcMotor.get("motorFRight");
            motorBLeft = hardwareMap.dcMotor.get("motorBLeft");
            motorBRight = hardwareMap.dcMotor.get("motorBRight");
            gyroMain = hardwareMap.gyroSensor.get("gyroMain");
            colorsensFLeft = hardwareMap.colorSensor.get("colorsensFLeft");
            colorsensLine = hardwareMap.opticalDistanceSensor.get("colorsensLine");
            distanceMain = hardwareMap.opticalDistanceSensor.get("distanceMain");
            servoLeft = hardwareMap.servo.get("servoLeft");
            servoRight = hardwareMap.servo.get("servoRight");

            motorFRight.setDirection(DcMotor.Direction.REVERSE);
            motorBRight.setDirection(DcMotor.Direction.REVERSE);

            motorFLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorFLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            getAutonomousParameters();
            gyroMain.calibrate();

            //telemetry.addData(name, " INITIALIZED.");
            telemetry.update();

            waitForStart();
            telemetry.clearAll();
            do{
                GYRO_HEADING = gyroMain.getHeading();
                telemetry.addData("Line Sensor Data:", colorsensLine.getRawLightDetected());
                telemetry.addData("Left Position", motorFLeft.getCurrentPosition());
                telemetry.addData("Right Position", motorFRight.getCurrentPosition());
                telemetry.addData("Left Power", motorFLeft.getPower());
                telemetry.addData("Right Power", motorFRight.getPower());
                telemetry.update();

                ASSMove(TILE * 4 , true);

                idle();
            }while(opModeIsActive());
            idle();
    }
}
