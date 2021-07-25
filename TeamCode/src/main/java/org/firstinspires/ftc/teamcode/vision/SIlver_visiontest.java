/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * In this sample, we demonstrate how to use the {@link OpenCvPipeline#onViewportTapped()}
 * callback to switch which stage of a pipeline is rendered to the viewport for debugging
 * purposes. We also show how to get data from the pipeline to your OpMode.
 */
@Disabled
@TeleOp
public class SIlver_visiontest extends LinearOpMode
{
    OpenCvCamera phoneCam;
    StageSwitchingPipeline stageSwitchingPipeline;
    private static Mat cvCvtcolorOutput = new Mat();
    private static Mat hsvThresholdOutput = new Mat();
    private static Mat cvErodeOutput = new Mat();
    private static Mat cvDilateOutput = new Mat();
    private static ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        stageSwitchingPipeline = new StageSwitchingPipeline();
        phoneCam.setPipeline(stageSwitchingPipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive())
        {

            telemetry.update();
            sleep(100);
        }
    }

    /*
     * With this pipeline, we demonstrate how to change which stage of
     * is rendered to the viewport when the viewport is tapped. This is
     * particularly useful during pipeline development. We also show how
     * to get data from the pipeline to your OpMode.
     */
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat contoursOnFrameMat = new Mat();
        ArrayList<MatOfPoint> contoursList = new ArrayList<>();
        int numContoursFound;
        private ArrayList<MatOfPoint> filterContoursOutput;

        enum Stage
        {
            YCbCr_CHAN2,
            THRESHOLD,
            CONTOURS_OVERLAYED_ON_FRAME,
            RAW_IMAGE,
        }

        private Stage stageToRenderToViewport = Stage.YCbCr_CHAN2;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat source0)
        {
            contoursList.clear();
            Mat cvCvtcolorSrc = source0;
            int cvCvtcolorCode = Imgproc.COLOR_RGB2HSV;
            cvCvtcolor(cvCvtcolorSrc, cvCvtcolorCode, cvCvtcolorOutput);

            // Step HSV_Threshold0:
            Mat hsvThresholdInput = cvCvtcolorOutput;
            double[] hsvThresholdHue = {129.6807432305976, 176.94693028095733};
            double[] hsvThresholdSaturation = {105.48561151079136, 135.33276450511946};
            double[] hsvThresholdValue = {255.0, 255.0};
            hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

            // Step CV_erode0:
            Mat cvErodeSrc = hsvThresholdOutput;
            Mat cvErodeKernel = new Mat();
            Point cvErodeAnchor = new Point(-1, -1);
            double cvErodeIterations = 6.0;
            int cvErodeBordertype = Core.BORDER_CONSTANT;
            Scalar cvErodeBordervalue = new Scalar(-1);
            cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

            // Step CV_dilate0:
            Mat cvDilateSrc = cvErodeOutput;
            Mat cvDilateKernel = new Mat();
            Point cvDilateAnchor = new Point(-1, -1);
            double cvDilateIterations = 17.0;
            int cvDilateBordertype = Core.BORDER_CONSTANT;
            Scalar cvDilateBordervalue = new Scalar(-1);
            cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);
            Mat findContoursInput = cvDilateOutput;
            boolean findContoursExternalOnly = true;
            findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

            // Step Filter_Contours0:
            ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
            double filterContoursMinArea = 20000.0;
            double filterContoursMinPerimeter = 0.0;
            double filterContoursMinWidth = 300.0;
            double filterContoursMaxWidth = 50000.0;
            double filterContoursMinHeight = 100.0;
            double filterContoursMaxHeight = 500.0;
            double[] filterContoursSolidity = {0, 100};
            double filterContoursMaxVertices = 1000000.0;
            double filterContoursMinVertices = 0.0;
            double filterContoursMinRatio = 0.9;
            double filterContoursMaxRatio = 1.5;
            filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
            source0.copyTo(contoursOnFrameMat);
            Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 3, 8);

            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */



            switch (stageToRenderToViewport)
            {
                case YCbCr_CHAN2:
                {
                    return yCbCrChan2Mat;
                }

                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case CONTOURS_OVERLAYED_ON_FRAME:
                {
                    return contoursOnFrameMat;
                }

                case RAW_IMAGE:
                {
                    return source0;
                }

                default:
                {
                    return source0;
                }
            }
        }

        /**
         * This method is a generated getter for the output of a CV_cvtColor.
         * @return Mat output from CV_cvtColor.
         */
        public Mat cvCvtcolorOutput() {
            return cvCvtcolorOutput;
        }

        /**
         * This method is a generated getter for the output of a HSV_Threshold.
         * @return Mat output from HSV_Threshold.
         */
        public Mat hsvThresholdOutput() {
            return hsvThresholdOutput;
        }

        /**
         * This method is a generated getter for the output of a CV_erode.
         * @return Mat output from CV_erode.
         */
        public Mat cvErodeOutput() {
            return cvErodeOutput;
        }

        /**
         * This method is a generated getter for the output of a CV_dilate.
         * @return Mat output from CV_dilate.
         */
        public Mat cvDilateOutput() {
            return cvDilateOutput;
        }

        /**
         * This method is a generated getter for the output of a Find_Contours.
         * @return ArrayList<MatOfPoint> output from Find_Contours.
         */
        public ArrayList<MatOfPoint> findContoursOutput() {
            return findContoursOutput;
        }

        /**
         * This method is a generated getter for the output of a Filter_Contours.
         * @return ArrayList<MatOfPoint> output from Filter_Contours.
         */
        public ArrayList<MatOfPoint> filterContoursOutput() {
            return filterContoursOutput;
        }


        /**
         * Converts an image from one color space to another.
         * @param src Image to convert.
         * @param code conversion code.
         * @param dst converted Image.
         */
        public void cvCvtcolor(Mat src, int code, Mat dst) {
            Imgproc.cvtColor(src, dst, code);
        }


        public void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                                  Mat out) {
            Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
            Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                    new Scalar(hue[1], sat[1], val[1]), out);
        }

        /**
         * Expands area of lower value in an image.
         * @param src the Image to erode.
         * @param kernel the kernel for erosion.
         * @param anchor the center of the kernel.
         * @param iterations the number of times to perform the erosion.
         * @param borderType pixel extrapolation method.
         * @param borderValue value to be used for a constant border.
         * @param dst Output Image.
         */
        private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
                             int borderType, Scalar borderValue, Mat dst) {
            if (kernel == null) {
                kernel = new Mat();
            }
            if (anchor == null) {
                anchor = new Point(-1,-1);
            }
            if (borderValue == null) {
                borderValue = new Scalar(-1);
            }
            Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
        }

        /**
         * Expands area of higher value in an image.
         * @param src the Image to dilate.
         * @param kernel the kernel for dilation.
         * @param anchor the center of the kernel.
         * @param iterations the number of times to perform the dilation.
         * @param borderType pixel extrapolation method.
         * @param borderValue value to be used for a constant border.
         * @param dst Output Image.
         */
        private void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
                              int borderType, Scalar borderValue, Mat dst) {
            if (kernel == null) {
                kernel = new Mat();
            }
            if (anchor == null) {
                anchor = new Point(-1,-1);
            }
            if (borderValue == null){
                borderValue = new Scalar(-1);
            }
            Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
        }


        private void findContours(Mat input, boolean externalOnly,
                                  List<MatOfPoint> contours) {
            Mat hierarchy = new Mat();
            contours.clear();
            int mode;
            if (externalOnly) {
                mode = Imgproc.RETR_EXTERNAL;
            }
            else {
                mode = Imgproc.RETR_LIST;
            }
            int method = Imgproc.CHAIN_APPROX_SIMPLE;
            Imgproc.findContours(input, contours, hierarchy, mode, method);
        }


        /**
//         * Filters out contours that do not meet certain criteria.
//         * @param inputContours is the input list of contours
//         * @param output is the the output list of contours
//         * @param minArea is the minimum area of a contour that will be kept
//         * @param minPerimeter is the minimum perimeter of a contour that will be kept
//         * @param minWidth minimum width of a contour
//         * @param maxWidth maximum width
//         * @param minHeight minimum height
//         * @param maxHeight maximimum height
//         * @param Solidity the minimum and maximum solidity of a contour
//         * @param minVertexCount minimum vertex Count of the contours
//         * @param maxVertexCount maximum vertex Count
//         * @param minRatio minimum ratio of width to height
//         * @param maxRatio maximum ratio of width to height
//         */
        private void filterContours(List<MatOfPoint> inputContours, double minArea,
                                    double minPerimeter, double minWidth, double maxWidth, double minHeight, double
                                            maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
                                            minRatio, double maxRatio, List<MatOfPoint> output) {
            final MatOfInt hull = new MatOfInt();
            output.clear();
            //operation
            for (int i = 0; i < inputContours.size(); i++) {
                final MatOfPoint contour = inputContours.get(i);
                final Rect bb = Imgproc.boundingRect(contour);
                if (bb.width < minWidth || bb.width > maxWidth) continue;
                if (bb.height < minHeight || bb.height > maxHeight) continue;
                final double area = Imgproc.contourArea(contour);
                if (area < minArea) continue;
                if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
                Imgproc.convexHull(contour, hull);
                MatOfPoint mopHull = new MatOfPoint();
                mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
                for (int j = 0; j < hull.size().height; j++) {
                    int index = (int)hull.get(j, 0)[0];
                    double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                    mopHull.put(j, 0, point);
                }
                final double solid = 100 * area / Imgproc.contourArea(mopHull);
                if (solid < solidity[0] || solid > solidity[1]) continue;
                if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
                final double ratio = bb.width / (double)bb.height;
                if (ratio < minRatio || ratio > maxRatio) continue;
                output.add(contour);
            }
        }
    }
}