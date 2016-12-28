package org.firstinspires.ftc.teamcode.FTC745Lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_2_DEV.lshootPower;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_2_DEV.motorLshoot;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_2_DEV.motorRshoot;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_2_DEV.rshootPower;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_2_DEV.servoShooterPipe;
import static org.firstinspires.ftc.teamcode.FTC745RobitTeleOp_v2_2_DEV.shootpipeMax;


public class FTC745Drive_v2_0_DEV{
    public abstract static class DriveTeleOp extends LinearOpMode{
        public static void Shoot(){
            if(motorLshoot.getPower() != 0.15 || motorRshoot.getPower() != 0.18){
                motorLshoot.setPower(lshootPower);
                motorRshoot.setPower(rshootPower);
            }
            servoShooterPipe.setPosition(shootpipeMax);
        }
  }
}