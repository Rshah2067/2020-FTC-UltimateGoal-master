package org.firstinspires.ftc.teamcode.mechexample;




import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;




public class Mech_machine {
    /* Public OpMode members. */
    public DcMotor intake = null;
    public DcMotor frontleft = null;
    public DcMotor frontright = null;
    public DcMotor backleft = null;
    public DcMotor backright = null;
    public CRServo slider = null;
    public Servo intakeangle = null;
    public DcMotorEx flywheelMotor1= null;
    public DcMotorEx flywheelMotor2 =null;
//    IntegratingGyroscope Ig;
//    public ModernRoboticsI2cGyro gyro = null;

    /* Local OpMode members. */
    HardwareMap hwMapm = null;
    public ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public Mech_machine() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMapm = ahwMap;


        // Define and Initialize Motors
        intake = hwMapm.get(DcMotor.class,"intake");
        frontleft = hwMapm.get(DcMotor.class, "frontleft");
        frontright = hwMapm.get(DcMotor.class, "frontright");
        backleft = hwMapm.get(DcMotor.class, "backleft");
        backright = hwMapm.get(DcMotor.class, "backright");
        slider = hwMapm.get(CRServo.class, "slider");
        flywheelMotor1=hwMapm.get(DcMotorEx.class,"flywheelMotor1");
        flywheelMotor2=hwMapm.get(DcMotorEx.class,"flywheelMotor2");
        intakeangle = hwMapm.get(Servo.class,"intakeangle");
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);
        //Set all motors to zero power
        frontleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);


        intakeangle.setPosition(0);



    }

    //sets the motors to run as they should in autonomous
    public void set_Motors_Autonomous (){
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);


        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void set_Motors_Teleop(){
        // Set all motors to run without encoders.
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);


        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

}