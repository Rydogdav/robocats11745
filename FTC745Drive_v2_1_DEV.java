package org.firstinspires.ftc.teamcode.FTC745Lib;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.FTC745RobitShootTestv1_0.servoShooterGate;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.Forward;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.Kf;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.Ks;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.Kt;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.Strafe;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.currentGear;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.currentHeading;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.gyroMain;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.maxMotorPower;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorBLeft;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorBRight;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorFLeft;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorFRight;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorLshoot;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.motorRshoot;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_3_DEV.servoShooterPipe;


public class FTC745Drive_v2_1_DEV {
    public abstract static class DriveTeleOp extends LinearOpMode {
        public static double motorFLeftv;
        public static double motorFRightv;
        public static double motorBLeftv;
        public static double motorBRightv;
        public static double motorFLeftFwd;
        public static double motorFRightFwd;
        public static double motorBLeftFwd;
        public static double motorBRightFwd;


        public static void FieldCentricMecanum(double North, double East, double TurnCW) {
            double strfOffset = 0.23;
            double fwdRevOffset = 0.95;
            //Convert from field-centric inputs to robot-centric commands
            currentHeading = gyroMain.getHeading();
            currentHeading = 0;
            Forward = +North * Math.cos(currentHeading) + East * Math.sin(currentHeading);
            Strafe = -North * Math.sin(currentHeading) + East * Math.cos(currentHeading);

            //Scaling outputs for gear input and tuning constants
            Forward = currentGear * Kf * Forward;
            Strafe = currentGear * Ks * Strafe;
            TurnCW = currentGear * Kt * TurnCW;

            //apply inverse kinematics to the scaled input
            motorFLeftv = +Forward + TurnCW + Strafe;
            motorFRightv = +Forward - TurnCW - Strafe;
            motorBLeftv = +Forward + TurnCW - Strafe;
            motorBRightv = +Forward - TurnCW + Strafe;

            if (motorFLeftv < 0) {
                motorFLeftv = motorFLeftv * motorFLeftFwd;
            }
            if (motorFRightv < 0) {
                motorFRightv = motorFRightv * motorFRightFwd;
            }
            if (motorBLeftv < 0) {
                motorBLeftv = motorBLeftv * motorBLeftFwd;
            }
            if (motorBRightv < 0) {
                motorBRightv = motorBRightv * motorBRightFwd;
            }
            if (motorFLeftv != 0) {
                motorFLeftv = motorFLeftv * 0.95;
            }
            if (motorBRightv != 0) {
                motorBRightv = motorBRightv * 0.95;
            }
            if (motorFRightv != 0) {

            }
            if (motorBLeftv != 0) {
                motorBLeftv = motorBLeftv * 1.3;
            }

                if (motorFLeft.getDirection() == DcMotorSimple.Direction.REVERSE) {
                    motorFLeftFwd = 1 - strfOffset;
                }
                if (motorFRight.getDirection() == DcMotorSimple.Direction.REVERSE) {
                    motorFRightFwd = 1 - strfOffset;
                }
                if (motorBLeft.getDirection() == DcMotorSimple.Direction.REVERSE) {
                    motorBLeftFwd = 1 - strfOffset;
                }
                if (motorBRight.getDirection() == DcMotorSimple.Direction.REVERSE) {
                    motorBRightFwd = 1 - strfOffset;
                }
                if (motorFLeft.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    motorFLeftFwd = 1 + strfOffset;
                }
                if (motorFRight.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    motorFRightFwd = 1 + strfOffset;
                }
                if (motorBLeft.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    motorBLeftFwd = 1 + strfOffset;
                }
                if (motorBRight.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    motorBRightFwd = 1 + strfOffset;
                }
            //find maximum input
            double maxInput = Math.abs(motorFLeftv);
            if (Math.abs(motorFRightv) > maxInput) maxInput = Math.abs(motorFRightv);
            if (Math.abs(motorBLeftv) > maxInput) maxInput = Math.abs(motorBLeftv);
            if (Math.abs(motorBRightv) > maxInput) maxInput = Math.abs(motorBRightv);


            //normalize to maximum allowed motor power
            if (maxInput > maxMotorPower) {
                motorFLeftv = maxMotorPower * motorFLeftv / maxInput;
                motorFRightv = maxMotorPower * motorFRightv / maxInput;
                motorBLeftv = maxMotorPower * motorBLeftv / maxInput;
                motorBRightv = maxMotorPower * motorBRightv / maxInput;
            }
        }
    }

    public abstract static class DriveAuton extends LinearOpMode {
        final static int WHEEL_DIAMETER = 4;     //Diameter of the wheel in inches
        final static double WHEEL_DIAMETER_MM = WHEEL_DIAMETER * (25.4);
        final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_MM;
        final static int ENCODER_CPR = 1120;     //Encoder Counts per Revolution
        final static double GEAR_RATIO = 1;      //Gear Ratio

        final static double ROBOT_TURN_CIRCLE_RADIUS = 7.625;
        final static double ROBOT_TURN_CURCUMFERECE = ROBOT_TURN_CIRCLE_RADIUS * Math.PI * 25.4;

        public static void AllStop() {
            motorBLeft.setPower(0);
            motorBRight.setPower(0);
            motorFLeft.setPower(0);
            motorFRight.setPower(0);
        }

        private static void HeadingTurn(double HEADING) {
            boolean move = true;
            do {
                Thetacurr = gyroMain.getHeading();
                if (HEADING <= 180 && Thetacurr >= 180) {
                    if (HEADING != Thetacurr) {
                        motorFLeft.setPower(-0.2); //Runs to position at this power
                        motorBLeft.setPower(-0.2);
                        motorFRight.setPower(0.2);
                        motorBRight.setPower(0.2);
                    } else {
                        AllStop();
                    }
                } else {
                    if (HEADING != Thetacurr) {
                        motorFLeft.setPower(0.2); //Runs to position at this power
                        motorBLeft.setPower(0.2);
                        motorFRight.setPower(-0.2);
                        motorBRight.setPower(-0.2);
                    } else {
                        AllStop();
                    }
                }
            } while (move);
        }

        public static void ResetEncoder() {
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

        public static void LinearMove(double DISTANCE, boolean FORWARDS) {
            final double ROTATIONS = DISTANCE / CIRCUMFERENCE;
            final double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
            int DIRECTION_MULTIPLIER;

            ResetEncoder();
            if (FORWARDS == true) {
                DIRECTION_MULTIPLIER = 1;
            } else {
                DIRECTION_MULTIPLIER = -1;
            }
            motorFLeft.setTargetPosition((int) COUNTS); //Sets position in counts
            motorFRight.setTargetPosition((int) COUNTS);
            motorBLeft.setTargetPosition((int) COUNTS);
            motorBRight.setTargetPosition((int) COUNTS);

            motorFLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to position
            motorFRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFLeft.setPower(DIRECTION_MULTIPLIER * 0.5 * DIRECTION_MULTIPLIER);
            motorFRight.setPower(DIRECTION_MULTIPLIER * 0.5 * DIRECTION_MULTIPLIER);
            motorBLeft.setPower(DIRECTION_MULTIPLIER * 0.5 * DIRECTION_MULTIPLIER);
            motorBRight.setPower(DIRECTION_MULTIPLIER * 0.5 * DIRECTION_MULTIPLIER);
        }

        public static double HEADING_TARGET;
        public static double GYRO_HEADING_NEW;

        public static void ASSMove(double DISTANCE, boolean FORWARDS) { //"AutomatedStabilitySystemMove"
            double HEADING_DELTA;
            if (Thetacurr > 180) {
                HEADING_TARGET = Thetacurr - 180;
            } else {
                HEADING_TARGET = Thetacurr + 180;
            }
            int DIRECTION_MULTIPLIER;
            double POWER_RIGHT = 0.5;
            double POWER_LEFT = 0.5;
            final double ROTATIONS = DISTANCE / CIRCUMFERENCE;
            final double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
            if (FORWARDS) {
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

                if (Thetacurr > 180) {
                    GYRO_HEADING_NEW = Thetacurr - 180;
                } else {
                    GYRO_HEADING_NEW = Thetacurr + 180;
                }
                if (GYRO_HEADING_NEW < HEADING_TARGET) {//If Statement logic is inverted due to the headings being set to the 90-270 side instead of the 270-90 side.
                    HEADING_DELTA = (HEADING_TARGET - GYRO_HEADING_NEW) / 10;
                    POWER_RIGHT = POWER_RIGHT + HEADING_DELTA;
                    POWER_LEFT = POWER_LEFT;
                }
                if (GYRO_HEADING_NEW > HEADING_TARGET) {
                    HEADING_DELTA = (GYRO_HEADING_NEW - HEADING_TARGET) / 10;
                    POWER_LEFT = POWER_LEFT + HEADING_DELTA;
                    POWER_RIGHT = POWER_RIGHT;
                }
                if (Thetacurr == HEADING_TARGET) {
                    POWER_LEFT = 0.5;
                    POWER_RIGHT = 0.5;
                }
                motorFLeft.setPower(DIRECTION_MULTIPLIER * POWER_LEFT);
                motorFRight.setPower(DIRECTION_MULTIPLIER * POWER_RIGHT);
                motorBLeft.setPower(DIRECTION_MULTIPLIER * POWER_LEFT);
                motorBRight.setPower(DIRECTION_MULTIPLIER * POWER_RIGHT);
                Thetacurr = gyroMain.getHeading();

                if (motorFLeft.getCurrentPosition() == COUNTS && motorFRight.getCurrentPosition() == COUNTS) {
                    move = false;
                }
            } while (move);
            AllStop();
        }

        public static int Xcurr;
        public static int Ycurr;
        public static int Thetacurr;

        public static void Fwd(int Xnew, int Ynew, int Headingfinal, boolean Forwards) {
            double Distance = 0;
            double Thetadelta = 0;
            if (Xnew != Xcurr && Ynew != Ycurr) {
                boolean Aligned;
                int Xdelta = Xnew - Xcurr;
                int Ydelta = Ynew - Ycurr;
                Distance = Math.hypot(Xdelta, Ydelta);
                Thetadelta = Math.atan2(Ydelta, Xdelta);
                Thetacurr = gyroMain.getHeading();
                if (Thetacurr > Thetadelta) {
                    HeadingTurn(Thetacurr - Thetadelta);
                } else {
                    HeadingTurn(Thetadelta - Thetacurr);
                }

                ASSMove(Distance, Forwards);
                HeadingTurn(Headingfinal);
            } else if (Xnew == Xcurr) {
                if (Thetadelta != Thetacurr) {
                    if (Forwards) {
                        HeadingTurn(0);
                        ASSMove(Distance, true);
                        HeadingTurn(Headingfinal);
                    } else {
                        HeadingTurn(180);
                        ASSMove(Distance, false);
                        HeadingTurn(Headingfinal);
                    }
                }
            } else if (Ynew == Ycurr) {
                if (Thetadelta != Thetacurr) {
                    if (Forwards) {
                        HeadingTurn(90);
                        ASSMove(Distance, true);
                        HeadingTurn(Headingfinal);
                    } else {
                        HeadingTurn(270);
                        ASSMove(Distance, false);
                        HeadingTurn(Headingfinal);
                    }
                }
            }
        }
    }

    public abstract static class Shooting extends LinearOpMode {
        public static void ParticleShootTele() {
            double shootpipeMax = 0.1;
            double shootpipeMin = 0.04;

            servoShooterPipe.setPosition(shootpipeMax);
            SystemClock.sleep(1000);
            servoShooterPipe.setPosition(shootpipeMin);
            servoShooterPipe.setPosition(shootpipeMax);
        }

        public static void ParticleShootAuton() {
            double lshootPower = 0.13;
            double rshootPower = 0.17;
            double shootpipeMax = 0.1;
            double shootpipeMin = 0.04;
            double shootgateMax = 0.27;
            double shootgateMin = 0.75;
            if (motorLshoot.getPower() != lshootPower || motorRshoot.getPower() != rshootPower) {
                motorLshoot.setPower(lshootPower);
                motorRshoot.setPower(rshootPower);
                SystemClock.sleep(3000);
            }
            servoShooterPipe.setPosition(shootpipeMax);
            SystemClock.sleep(1000);
            servoShooterPipe.setPosition(shootpipeMin);
            servoShooterGate.setPosition(shootgateMax);
            SystemClock.sleep(1000);
        }
    }
}
