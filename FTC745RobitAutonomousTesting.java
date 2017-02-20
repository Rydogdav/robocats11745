package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_RELEASE.DriveAuton.ASSMove;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_RELEASE.DriveAuton.Fwd;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_RELEASE.DriveAuton.Xcurr;
import static org.firstinspires.ftc.teamcode.FTC745Lib.FTC745Drive_v2_0_RELEASE.DriveAuton.Ycurr;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_2_RELEASE.TILE;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_2_RELEASE.gyroMain;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_2_RELEASE.motorBLeft;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_2_RELEASE.motorBRight;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_2_RELEASE.motorFLeft;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_2_RELEASE.motorFRight;

/**
 * Created by Ryan Davitt on 1/4/2017.
 */
@Autonomous(name ="Auton Testing v1.0", group ="Autonomous")
public class FTC745RobitAutonomousTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        gyroMain = hardwareMap.gyroSensor.get("gyroMain");
        motorFLeft = hardwareMap.dcMotor.get("motorFLeft");
        motorFRight = hardwareMap.dcMotor.get("motorFRight");
        motorBLeft = hardwareMap.dcMotor.get("motorBLeft");
        motorBRight = hardwareMap.dcMotor.get("motorBRight");
        gyroMain.calibrate();
        telemetry.addLine("1.0 Initilized");
        idle();
        waitForStart();
        ASSMove(TILE * 2, true);
        /*Xcurr = 0;
        Ycurr = 0;
        Fwd(TILE * 2, TILE, 0, true);*/
        idle();
        requestOpModeStop();
    }
}
