package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
@Disabled
public class Shooter extends LinearOpMode {
    public DcMotorEx Shooterwheel = null;
    public DcMotorEx Shooterpully = null;
    public ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Shooterwheel = hardwareMap.get(DcMotorEx.class,"Shooterwheel");
        Shooterpully = hardwareMap.get(DcMotorEx.class,"shooterpully");
        Shooterpully.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooterwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elapsedTime.reset();
        while(opModeIsActive() &&!isStopRequested() ){


            if(gamepad1.a){
                elapsedTime.reset();
                Shooterpully.setMotorEnable();
                Shooterwheel.setMotorEnable();
                Shooterpully.setVelocity(ticks_rpm(3000));
                Shooterwheel.setVelocity(ticks_rpm(3000));

            }

            if (gamepad1.b){
               Shooterwheel.setMotorDisable();
               Shooterpully.setMotorDisable();
            }
            telemetry.addData("power",Shooterpully.getPower());
            telemetry.addData("velocity",Shooterpully.getVelocity());
            telemetry.addData("ticks",Shooterpully.getCurrentPosition());
            telemetry.update();

        }
    }
    public int ticks_rpm(int velocity){
        int ticks = (velocity)*(28/60);
        return ticks;
    }
}

