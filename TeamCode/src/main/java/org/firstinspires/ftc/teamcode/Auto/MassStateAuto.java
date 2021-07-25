package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.qual2.qual2_machine;
import org.firstinspires.ftc.teamcode.vision.WebcamExample;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
@Autonomous
@Config
public class MassStateAuto extends LinearOpMode {

//    qual2_machine robot = new qual2_machine();
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime opmodetime = new ElapsedTime();

    OpenCvCamera webcam;
    private Mat Cb = new Mat();

    public static int x,y,width,height;
    private Rect submatRect = new Rect(1, 95, 55, 45);
    private Rect submatRect2 = new Rect(1,120,55,20);
    public  static int avg1,avg2;
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
    public DcMotor flywheelMotor1= null;
    public DcMotor flywheelMotor2 =null;

    int ringposition;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotor.class,"intake");

        slider = hardwareMap.get(Servo.class, "slider");
        flywheelMotor1=hardwareMap.get(DcMotor.class,"flywheelMotor1");
        flywheelMotor2=hardwareMap.get(DcMotor.class,"flywheelMotor2");
        intakeangle = hardwareMap.get(Servo.class,"intakeangle");
        wobblearm = hardwareMap.get(Servo.class,"wobblearm");
        wobblegrip = hardwareMap.get(Servo.class,"wobblegrip");
        flapservo = hardwareMap.get(Servo.class, "flapservo");

        //Set all motors to zero power

        flapservo.setPosition(1);
        slider.setPosition(0);
        intakeangle.setPosition(0);
        wobblearm.setPosition(1);
        wobblegrip.setPosition(1);
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        startpipeline();
        Pose2d startingpos = new Pose2d(-63,26.5,Math.toRadians(0));
        drive.setPoseEstimate(startingpos);
        webcam.setPipeline(new SamplePipeline());
        Pose2d poseEstimate = drive.getPoseEstimate();
        Trajectory traj10stack = drive.trajectoryBuilder(startingpos)
                .splineTo(new Vector2d(-17,22),Math.toRadians(356))
                .build();

        while (!isStarted()){
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("avg1",avg1);
            telemetry.addData("avg2",avg2);
            telemetry.addData("ringposition",ringposition);
            telemetry.update();
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();

        opmodetime.reset();
        if (!(opmodetime.seconds() >= 30) &&!isStopRequested()){

            wobblearm.setPosition(.79);
            sleep(800);

            wobblegrip.setPosition(.28);
            sleep(800);
            wobblearm.setPosition(0);
            flywheelMotor1.setPower(1);
            flywheelMotor2.setPower(1);
            drive.followTrajectory(traj10stack);
            sleep(50);
            slider.setPosition(.4);
            sleep(60);
            drive.turn(Math.toRadians(-7.2));
            slider.setPosition(0);
            sleep(70);
            slider.setPosition(.4);
            sleep(60);
            drive.turn(Math.toRadians(-8.8));
            slider.setPosition(0);

            sleep(80);
            slider.setPosition(.4);

            Trajectory traj20stack = drive.trajectoryBuilder(new Pose2d(-18,22), poseEstimate.getHeading())
                    .splineTo(new Vector2d(17,40),Math.toRadians(0))
                    .build();

            Trajectory traj30stack = drive.trajectoryBuilder(traj20stack.end(),poseEstimate.getHeading())
                    .lineTo(new Vector2d(-46,35))
                    .build();
            Trajectory traj40stack = drive.trajectoryBuilder(traj30stack.end(),poseEstimate.getHeading())
                    .strafeLeft(4)
                    .build();
            Trajectory traj50stack = drive.trajectoryBuilder(traj40stack.end(), poseEstimate.getHeading())
                    .splineTo(new Vector2d(21,40),Math.toRadians(0))
                    .build();
            Trajectory traj21stack = drive.trajectoryBuilder(new Pose2d(-18,22),poseEstimate.getHeading())
                    .splineTo(new Vector2d(30,22),Math.toRadians(0))
                    .build();
            Trajectory traj31stack = drive.trajectoryBuilder(traj21stack.end(),poseEstimate.getHeading())
                    .back(77.5)
                    .build();
            Trajectory traj41stack = drive.trajectoryBuilder(traj31stack.end(),poseEstimate.getHeading())
                    .strafeLeft(16)
                    .build();
            if (ringposition == 0){
                flywheelMotor1.setPower(0);
                flywheelMotor2.setPower(0);
                drive.followTrajectory(traj20stack);

                wobblearm.setPosition(.7);
                wobblegrip.setPosition(1);
                sleep(80);
                wobblearm.setPosition(1);
                drive.followTrajectory(traj30stack);
                sleep(80);
                drive.followTrajectory(traj40stack);
                sleep(80);
                wobblearm.setPosition(.79);
                sleep(800);
                wobblegrip.setPosition(.28);
                sleep(800);
                wobblearm.setPosition(0);

                sleep(80);
                drive.followTrajectory(traj50stack);
                sleep(90);
                wobblearm.setPosition(.82);
                sleep(900);
                wobblegrip.setPosition(1);
                sleep(100);
            }
            if (ringposition == 1){
                flywheelMotor1.setPower(0);
                flywheelMotor2.setPower(0);
                drive.followTrajectory(traj21stack);
                wobblearm.setPosition(82);
                sleep(800);
                wobblegrip.setPosition(1);
                sleep(500);
                drive.followTrajectory(traj31stack);
                sleep(80);
                drive.followTrajectory(traj41stack);
                wobblearm.setPosition(.79);
                sleep(800);
                wobblegrip.setPosition(.28);
                sleep(800);
                wobblearm.setPosition(0);
            }
            sleep(1000);

        }

    }
    public void startpipeline(){
        FtcDashboard.getInstance().startCameraStream(webcam,0);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });


    }

    void inputcvt(Mat input){
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(input,Cb,2);
    }
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        Mat Cropped;
        Mat Croppedtwo;

        @Override
        public Mat processFrame(Mat input)
        {
            inputcvt(input);
            Cropped = input.submat(submatRect);
            Croppedtwo = input.submat(submatRect2);
            Imgproc.rectangle(input,submatRect,new Scalar(0,0,255),3);
            /* Executed each frame, the returned mat will be the one displayed */
            /* Processing and detection stuff */
            avg1 = (int) Core.mean(Cropped).val[2];
            avg2= (int) Core.mean(Croppedtwo).val[2];
            if (avg1 <= 130 && avg1 >= 120) {
                ringposition = 4;
            }
            else if (avg2 >=135 && avg2<=150){
                ringposition =1;
            }
            else if (avg2 >=155&& avg2<=165){
                ringposition = 0;
            }

            Cropped.release();
            Croppedtwo.release();
            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }

    }

}

