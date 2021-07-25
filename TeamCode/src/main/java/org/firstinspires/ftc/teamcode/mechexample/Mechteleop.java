package org.firstinspires.ftc.teamcode.mechexample;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import org.apache.commons.math3.filter.KalmanFilter;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**
 * Created on 12/24/2016.
 */
@Disabled
@Config
@TeleOp(name = "Mechteleop")
public class Mechteleop extends LinearOpMode {
    Mech_machine robot = new Mech_machine();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override

    public void runOpMode() throws InterruptedException {



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

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
        double errorAngle = 0;
        float powerPercentage = 0;
        double powerMultiplier = 1;
        double intakeangle =0;
        // Declare Sensors

        //gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        /*
        *************************************************************************************
        run until the end of the match (driver presses STOP)
        */
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



            telemetry.addData("rightstreight",robot.frontright.getCurrentPosition());
            telemetry.addData("sideways",robot.frontleft.getCurrentPosition());
            telemetry.addData("leftstreight",robot.backright.getCurrentPosition());
            telemetry.addData("velocity",robot.flywheelMotor1.getVelocity());
            telemetry.addData("intake servo",robot.intakeangle.getPosition());
            telemetry.update();

            /*******************
             * GAMEPAD TWO
             ******************/
                if (gamepad2.right_trigger >= .9){
                    robot.slider.setPower(.3);
                }
                else {
                    robot.slider.setPower(0);
                }
                if (gamepad2.x){
                    //    robot.flywheelMotor1.setVelocity(5800,AngleUnit.valueOf());
                   robot.flywheelMotor2.setVelocity(5800);

                }
                if (gamepad2.y){
                    robot.flywheelMotor2.setVelocity(0);
                    robot.flywheelMotor1.setVelocity(0);
                }
                if (gamepad2.a){
                    intakeangle = intakeangle+.1;
                    robot.intakeangle.setPosition(robot.intakeangle.getPosition()+.01);
                }
                if (gamepad2.dpad_up){
                    robot.intake.setPower(1);
                }
                else if (gamepad2.dpad_down){
                    robot.intake.setPower(-1);
                }
                else {
                    robot.intake.setPower(0);
            }


        }
    }
    public int tpstorpm(int ticks) {
        int velocity = 0;
        return velocity;
    }
}