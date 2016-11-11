/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop Robit Test Bed", group="Test Bed")  // @Autonomous(...) is the other common choice
//@Disabled
public class teleOpTestRobit extends OpMode
{
    /* Declare OpMode members. */
    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;
    public DcMotor motorBLeft = null;
    public DcMotor motorBRight = null;
    //public Servo servoMain = null;
    public GyroSensor gyroMain = null;

    /*public final static double homeServoMain = 0.0;
    public final static double minrngServoMain  = 0.0; //Min adjusted, just in case it dont work
    public final static double maxrngServoMain  = 1;
    public final double speedServoMain = 0.02 ;
    public double positionMain = 0;// sets rate to move servo*/

    public double joystickRight = 0;
    public double joystickLeft = 0;
    public double gearLow = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        motorLeft  = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorBLeft = hardwareMap.dcMotor.get("motorBLeft");
        motorBRight = hardwareMap.dcMotor.get("motorBRight");
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //servoMain = hardwareMap.servo.get("servoMain");
        gyroMain = hardwareMap.gyroSensor.get("gyroMain");

        telemetry.addData("Status", "Initialized. Welcome user. v2.2.1 Now with SERVOS (and Debug Gyro)!");
        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //servoMain.setPosition(homeServoMain);
        //positionMain = servoMain.getPosition();
        // telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
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
        telemetry.addData("Status", "Good luck! Running: " + runtime.toString());
        joystickRight = -gamepad1.right_stick_y;
        joystickLeft = -gamepad1.left_stick_y;
        if (gamepad1.right_trigger == 1) {
            gearLow =  1;
        }
        if(gamepad1.left_trigger == 1) {
            gearLow = 0;
        }
        if(gearLow == 1) {
            joystickRight = joystickRight * 0.2;
            joystickLeft = joystickLeft * 0.2;
        }
        motorLeft.setPower(joystickLeft);
        motorRight.setPower(joystickRight);
        motorBLeft.setPower(joystickLeft);
        motorBRight.setPower(joystickRight);
            /*if (gamepad1.right_bumper) {
                positionMain = servoMain.getPosition() + speedServoMain;
            }
            if (gamepad1.left_bumper) {
                positionMain = servoMain.getPosition() - speedServoMain;
            }
            if (gamepad1.b){
                servoMain.setPosition(homeServoMain);
                positionMain = homeServoMain;
            }
            if(positionMain < minrngServoMain){
                positionMain = .0;
            }
            if(positionMain > maxrngServoMain){
                positionMain = 1;8
            }*/
            if(gamepad1.right_trigger == 1) {
                gearLow = 1;
            }


            //servoMain.setPosition(positionMain);

            //telemetry.addData("DEBUG: servoMain Var:", positionMain);
            telemetry.addData("DEBUG: Gyroscope Heading:", gyroMain.getHeading());
            telemetry.addData("DEBUG: Gyroscope X:", gyroMain.rawX());
            telemetry.addData("DEBUG: Gyroscope Y:", gyroMain.rawY());
            telemetry.addData("DEBUG: Gyroscope Z:", gyroMain.rawZ());
            telemetry.update();


        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        // leftMotor.setPower(-gamepad1.left_stick_y);
        // rightMotor.setPower(-gamepad1.right_stick_y);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
