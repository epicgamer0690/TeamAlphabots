package org.firstinspires.ftc.teamcode.Movement_Sensor_Test.sensor_test.Sensors_test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 */

@Autonomous(name = "Freight Determination", group = "Training")
public class FreightDetermination extends LinearOpMode
{
    OpenCvWebcam webcam;
    FreightDeterminationPipeline pipeline;
    FreightDeterminationPipeline.FreightPosition snapshotAnalysis = FreightDeterminationPipeline.FreightPosition.LEFT; // default

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
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new FreightDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis)
        {
            case LEFT:
            {
                /* Your autonomous code */
                break;
            }
            case RIGHT:
            {
                break;
            }

            case CENTER:
            {
                
                break;
            }
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive())
        {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
    public static class FreightDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the Freight position
         */
        public enum FreightPosition
        {
            LEFT,
            CENTER,
            RIGHT,
            NOT_FOUND
        }

        /*
         * Some color constants for boxes
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(109,98);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(181,98);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(253,98);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        static final Rect LEFT_ROI = new Rect(
                new Point(REGION1_TOPLEFT_ANCHOR_POINT.x,
                        REGION1_TOPLEFT_ANCHOR_POINT.y),
                new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                        REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT));

        static final Rect CENTER_ROI = new Rect(
                new Point(REGION2_TOPLEFT_ANCHOR_POINT.x,
                        REGION2_TOPLEFT_ANCHOR_POINT.y),
                new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                        REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT));

        static final Rect RIGHT_ROI = new Rect(
                new Point(REGION3_TOPLEFT_ANCHOR_POINT.x,
                        REGION3_TOPLEFT_ANCHOR_POINT.y),
                new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                        REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT));
        /*
         * Working variables
         */
        Mat mat = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile FreightPosition position = FreightPosition.LEFT;

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Scalar lowHSV = new Scalar(23, 50, 70);
            Scalar highHSV = new Scalar(32, 255, 255);

            Core.inRange(mat, lowHSV, highHSV, mat);

            Mat left = mat.submat(new Rect(region1_pointA, region1_pointB));
            Mat center = mat.submat(new Rect(region2_pointA, region2_pointB));
            Mat right = mat.submat(new Rect(region3_pointA, region3_pointB));

            avg1 = (int) Core.mean(left).val[0];
            avg2 = (int) Core.mean(center).val[0];
            avg3 = (int) Core.mean(right).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            int minOneTwo = Math.min(avg1, avg2);
            int min = Math.min(minOneTwo, avg3);

            /*
             * Now that we found the min, we actually need to go and
             * figure out which sample region that value was from
             */
            if(min == avg1) // Was it from region 1?
            {
                position = FreightPosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(min == avg2) // Was it from region 2?
            {
                position = FreightPosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(min == avg3) // Was it from region 3?
            {
                position = FreightPosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }else{
                position = FreightPosition.NOT_FOUND;

                Imgproc.rectangle(
                        input,
                        region1_pointA,
                        region1_pointB,
                        RED,
                        -1);
                Imgproc.rectangle(
                        input,
                        region2_pointA,
                        region2_pointB,
                        RED,
                        -1);
                Imgproc.rectangle(
                        input,
                        region3_pointA,
                        region3_pointB,
                        RED,
                        -1);


            }

            return input;
        }


        public FreightPosition getAnalysis()
        {
            return position;
        }
    }
}
