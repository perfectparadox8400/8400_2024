package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Cube Detection Red")
public class red extends LinearOpMode {
    OpenCvCamera webcam;
    CubeDetectionPipeline pipeline;


    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        setupCam();

        waitForStart();
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        while (opModeIsActive()) {
            telemetry.addData("Left Red Value", CubeDetectionPipeline.getLeftRedValue());
            telemetry.addData("Center Red Value", CubeDetectionPipeline.getCenterRedValue());
            telemetry.addData("Right Red Value", CubeDetectionPipeline.getRightRedValue());
            telemetry.addData("Position", CubeDetectionPipeline.getPosition());
            telemetry.addData("THRESHOLD", CubeDetectionPipeline.getThreshold());
            packet.put("Left Red Value", CubeDetectionPipeline.getLeftRedValue());
            packet.put("Center Red Value", CubeDetectionPipeline.getCenterRedValue());
            packet.put("Right Red Value", CubeDetectionPipeline.getRightRedValue());
            packet.put("Position", CubeDetectionPipeline.getPosition());
            packet.put("THRESHOLD", CubeDetectionPipeline.getThreshold());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

    public void setupCam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CubeDetectionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public static class CubeDetectionPipeline extends OpenCvPipeline {
        Mat mat = new Mat();
        Mat redMask = new Mat();

        private static double leftRedValue = 0.0;
        private static double centerRedValue = 0.0;
        private static double rightRedValue = 0.0;

        private static double THRESHOLD = 5000.0;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

            // Red color range
            // Define two ranges for red because it wraps around the hue spectrum
            Scalar lowerRed1 = new Scalar(0, 100, 100); // Lower bound for red in HSV format
            Scalar upperRed1 = new Scalar(10, 255, 255); // Upper bound for red in HSV format

            Scalar lowerRed2 = new Scalar(160, 100, 100); // Lower bound for red in HSV format
            Scalar upperRed2 = new Scalar(180, 255, 255); // Upper bound for red in HSV format

            Mat redMask1 = new Mat();
            Mat redMask2 = new Mat();

            // Apply the inRange function for the first red range
            Core.inRange(mat, lowerRed1, upperRed1, redMask1);

            // Apply the inRange function for the second red range
            Core.inRange(mat, lowerRed2, upperRed2, redMask2);

            // Combine the masks for both ranges
            Core.bitwise_or(redMask1, redMask2, redMask);

            // Divide the frame into three sections (left, center, right)
            Rect leftRect = new Rect(0, mat.rows() / 3, mat.cols() / 4, mat.rows() - mat.rows() / 2);
            Rect centerRect = new Rect((mat.cols() / 3) + (mat.rows() / 24), mat.rows() / 3, mat.cols() / 4, mat.rows() - mat.rows() / 2);
            Rect rightRect = new Rect((2 * mat.cols() / 3) + mat.cols() / 12, mat.rows() / 3, mat.cols() / 4, mat.rows() - mat.rows() / 2);

            Mat left = mat.submat(leftRect);
            Mat center = mat.submat(centerRect);
            Mat right = mat.submat(rightRect);

            // Calculate the sum of red pixel values for each section
            leftRedValue = Core.sumElems(left).val[0];
            centerRedValue = Core.sumElems(center).val[0];
            rightRedValue = Core.sumElems(right).val[0];

            left.release();
            center.release();
            right.release();

            // Draw rectangles around the regions
            drawBox(input, leftRect, (leftRedValue > THRESHOLD) ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0)); // Green box if red is high, otherwise red
            drawBox(input, centerRect, (centerRedValue > THRESHOLD) ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0)); // Green box if red is high, otherwise red
            drawBox(input, rightRect, (rightRedValue > THRESHOLD) ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0)); // Green box if red is high, otherwise red

            return input;
        }

        // Get the red pixel value for the left section
        public static double getLeftRedValue() {
            return leftRedValue;
        }

        // Get the red pixel value for the center section
        public static double getCenterRedValue() {
            return centerRedValue;
        }

        // Get the red pixel value for the right section
        public static double getRightRedValue() {
            return rightRedValue;
        }
        public static double getThreshold() {
            return THRESHOLD;
        }

        // Increase the threshold value
        public static void increaseThreshold() {
            THRESHOLD += 500.0; // Adjust the step as needed
        }

        // Decrease the threshold value
        public static void decreaseThreshold() {
            THRESHOLD -= 500.0; // Adjust the step as needed
            if (THRESHOLD < 0.0) {
                THRESHOLD = 0.0;
            }
        }

        // Get the position based on the region with the highest red pixel value
        // Get the position based on the region with the highest red pixel value
        public static String getPosition() {
            if (leftRedValue > THRESHOLD && leftRedValue > centerRedValue && leftRedValue > rightRedValue) {
                return "Left";
            } else if (centerRedValue > THRESHOLD && centerRedValue > leftRedValue && centerRedValue > rightRedValue) {
                return "Center";
            } else if (rightRedValue > THRESHOLD && rightRedValue > leftRedValue && rightRedValue > centerRedValue) {
                return "Right";
            } else if (leftRedValue + centerRedValue + rightRedValue > THRESHOLD) {
                // If the sum of red pixel values in all sections is above the threshold, but none of the individual sections is high enough
                return "Unknown";
            } else {
                return "Unknown";
            }
        }

        private void drawBox(Mat input, Rect rect, Scalar color) {
            Imgproc.rectangle(input, rect.tl(), rect.br(), color, 2);
        }
    }
}