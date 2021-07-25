package org.firstinspires.ftc.teamcode.qual2;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class qual2_machine {
    /* Public OpMode members. */
    public DcMotor intake = null;
    public DcMotor frontleft = null;
    public DcMotor frontright = null;
    public DcMotor backleft = null;
    public DcMotor backright = null;
    public Servo slider = null;
    public Servo intakeangle = null;
    public Servo wobblearm = null;
    public Servo wobblegrip = null;
    public Servo flapservo = null;
    public DcMotorEx flywheelMotor1= null;
    public DcMotorEx flywheelMotor2 =null;

//    IntegratingGyroscope Ig;
//    public ModernRoboticsI2cGyro gyro = null;

    /* Local OpMode members. */
    HardwareMap hwMapm = null;
    public ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public qual2_machine() {
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
        slider = hwMapm.get(Servo.class, "slider");
        flywheelMotor1=hwMapm.get(DcMotorEx.class,"flywheelMotor1");
        flywheelMotor2=hwMapm.get(DcMotorEx.class,"flywheelMotor2");
        intakeangle = hwMapm.get(Servo.class,"intakeangle");
        wobblearm = hwMapm.get(Servo.class,"wobblearm");
        wobblegrip = hwMapm.get(Servo.class,"wobblegrip");
        flapservo = hwMapm.get(Servo.class, "flapservo");
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);
        //Set all motors to zero power
        frontleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        flapservo.setPosition(1);
        slider.setPosition(0);
        intakeangle.setPosition(0);
        wobblearm.setPosition(1);
        wobblegrip.setPosition(1);
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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