package org.firstinspires.ftc.teamcode.qual2;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TuningController;
import org.firstinspires.ftc.teamcode.VelocityPIDFController;
import org.firstinspires.ftc.teamcode.mechexample.Mech_machine;


/**
 * Created on 12/24/2016.
 */

@TeleOp(name = "Qual2teleop")
public class Qual2teleop extends LinearOpMode {
    qual2_machine robot = new qual2_machine();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.00016, 0, 00002);

    // Copy your feedforward gains here
    public static double kV = 1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0.0000003;
    public static double kStatic = 0;

    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect
    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    // Our velocity controller
    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        float powerPercentage = 0;
        double powerMultiplier = 1;
        boolean inshootingposition =false;
        boolean slidermoving =false;
        int wobblegoaldown = 1;
        boolean wobblegoalgrip = false;
        ElapsedTime wobblearmtime = new ElapsedTime();
        ElapsedTime servotime= new ElapsedTime();
        ElapsedTime wobblegriptime = new ElapsedTime();
        boolean shooteron = false;
        ElapsedTime shootertime = new ElapsedTime();
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        double targetVelo = 0.0;

        robot.init(hardwareMap);


        // Set Motor Directions (Set differently in Autonomous)
         robot.backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.backright.setDirection(DcMotorSimple.Direction.FORWARD);


        // Use FLOAT to make drive motors soft stop
        robot.backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //Initialize Gyro
//        ModernRoboticsI2cGyro gyro;   // Hardware Device Object
//        int xVal, yVal, zVal = 0;     // Gyro rate Values
//        int heading = 0;              // Gyro integrated heading
//        int angleZ = 0;
//        boolean lastResetState = false;
//        boolean curResetState  = false;
//
//        // get a reference to a Modern Robotics GyroSensor object.
//        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
//
//        // start calibrating the gyro.
//        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
//        telemetry.update();
//        gyro.calibrate();
//
//        // make sure the gyro is calibrated.
//        while (!isStopRequested() && gyro.isCalibrating())  {
//            sleep(50);
//            idle();
//        }
//
//        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
//        telemetry.update();

        // Initialize Variables



        // Declare Sensors

        //gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        /*
        *************************************************************************************
        run until the end of the match (driver presses STOP)
        */
        servotime.reset();
        while (opModeIsActive()) {

            /*******************
             * DECLARATIONS
             ******************/
            // Declare Variables and Servo Directions

            //errorAngle = gyro.getIntegratedZValue();


            /*******************
             * GAMEPAD ONE
             ******************/

            // Set Gamepad 1 Joysticks to Variables
            double horizontalComponent = -(gamepad1.left_stick_x);
            double upComponent = gamepad1.left_stick_y;
            double turnComponent = gamepad1.right_stick_x;

            //Soft Start
            if (horizontalComponent == 0 && upComponent == 0 && turnComponent == 0) {
                powerPercentage = (float) (0);
            }
            if (powerPercentage < 1) {
                powerPercentage = (float) (powerPercentage + .1);
            }

            if (gamepad1.left_trigger >= .9) {   //Softstop Off
                robot.backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }else{
                robot.backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            if (gamepad1.left_bumper){
                powerMultiplier = 0.25;
            }else{
                powerMultiplier = 1;
            }

            //TeleOp with Gyro to hold angle
            robot.frontright.setPower(((powerPercentage *powerMultiplier * (upComponent - horizontalComponent + turnComponent))));
            robot.frontleft.setPower(-((powerPercentage * powerMultiplier * (horizontalComponent + upComponent - turnComponent))));
            robot.backright.setPower(((powerPercentage * powerMultiplier* (horizontalComponent + upComponent + turnComponent))));
            robot.backleft.setPower(-((powerPercentage * powerMultiplier*(upComponent - horizontalComponent - turnComponent))));



            telemetry.addData("frontright",robot.frontright.getCurrentPosition());
            telemetry.addData("sideways",robot.frontleft.getCurrentPosition());
            telemetry.addData("left",robot.backleft.getCurrentPosition());
            telemetry.addData("velocity",robot.flywheelMotor1.getVelocity());
            telemetry.addData(" flapservo",robot.flapservo.getPosition());
           telemetry.addData("shooter servo",robot.slider.getPosition());
           telemetry.addData("wobblearm",robot.wobblearm.getPosition());
           telemetry.addData("wobblegrip",robot.wobblegrip.getPosition());
           telemetry.addData("wobblegriptime",wobblearmtime);
            telemetry.update();

            /*******************
             * GAMEPAD TWO
             ******************/
            if (gamepad2.right_trigger >=.9){
                robot.slider.setPosition(.4);
            }
            if (gamepad2.right_bumper){
                robot.slider.setPosition(0);
            }
//                if ((gamepad2.right_trigger >= .9 && )){
//                    servotime.reset();
//                    robot.slider.setPosition(.17);
// 0
//                }
//                if (gamepad2.right_trigger>=.9 && !gamepad2.right_bumper &&!slidermoving &&!inshootingposition){
//                    robot.slider.setPosition(.2);
//                    slidermoving = true;
//                }
//                if (slidermoving && !inshootingposition &&robot.slider.getPosition() <=.29 &&robot.slider.getPosition()>=.21){
//                    slidermoving = false;
//                    inshootingposition = true;
//                }
//                if (gamepad2.right_trigger>=.9 && !gamepad2.right_bumper &&!slidermoving &&inshootingposition){
//                    robot.slider.setPosition(0);
//                    slidermoving = true;
//                }
//                if (slidermoving && inshootingposition &&robot.slider.getPosition() <=.09&&robot.slider.getPosition()>=0){
//                    slidermoving = false;
//                    inshootingposition = false;
//                }
                 if(gamepad2.y &&servotime.milliseconds() >=500){
                     robot.flapservo.setPosition(robot.flapservo.getPosition()-.005);
                     servotime.reset();
                 }


            if (gamepad2.x && !shooteron && shootertime.milliseconds() >=500){
                    robot.flywheelMotor1.setVelocity(2700);
                    robot.flywheelMotor2.setVelocity(2700);
                   shootertime.reset();
                    shooteron = true;
                }
                if (gamepad2.x && shooteron &&shootertime.milliseconds() >=500){
                    robot.flywheelMotor1.setVelocity(0);
                    robot.flywheelMotor2.setVelocity(0);
                    shootertime.reset();
                    shooteron = false;
                }

            if (gamepad2.a ){
                    robot.intakeangle.setPosition(.35);
                    //position ~~.37
                }
            if (gamepad2.left_trigger >= .99 && wobblegoaldown ==1 && wobblearmtime.milliseconds()>=700 ){
                    robot.wobblearm.setPosition(.79);
                    wobblegoaldown = 2;
                    wobblearmtime.reset();
                    //.35
                }
            if (gamepad2.left_trigger >=.99 && wobblegoaldown==3 && wobblearmtime.milliseconds()>=700){
                    robot.wobblearm.setPosition(1);
                    wobblearmtime.reset();
                    wobblegoaldown =1;
                }
            if (gamepad2.left_trigger >=.9 && wobblegoaldown==2 && wobblearmtime.milliseconds()>=700){
                robot.wobblearm.setPosition(0);
                wobblearmtime.reset();
                wobblegoaldown =3;
            }

            if (gamepad2.left_bumper &&!wobblegoalgrip &&wobblegriptime.milliseconds()>=700){
                robot.wobblegrip.setPosition(1);
                wobblegoalgrip = true;
                wobblegriptime.reset();
            }
            if (gamepad2.left_bumper && wobblegoalgrip && wobblegriptime.milliseconds()>=700){
                    robot.wobblegrip.setPosition(.28);
                    wobblegoalgrip = false;
                    wobblegriptime.reset();
                }
            if (gamepad2.b){
                    robot.intakeangle.setPosition(0);
                }
            if (gamepad2.dpad_up){
                    robot.intake.setPower(1);
                }
            if (gamepad2.dpad_right) {
                robot.intakeangle.setPosition(.2);
            } else if (gamepad2.dpad_down) {
                robot.intake.setPower(-1);
            } else if (gamepad2.dpad_left) {
                robot.intake.setPower(0);
            }


        }
    }
    public double tpstorpm(double Velocity) {
//  28 ticks per revolution
        double ticks  = (Velocity/60)*28;
        return ticks;
    }
}