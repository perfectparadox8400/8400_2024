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

@Autonomous(name = "Cube Detection Blue")
public class blue extends LinearOpMode {
    OpenCvCamera webcam;
    CubeDetectionPipeline pipeline;


    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        setupCam();

        waitForStart();
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        while (opModeIsActive()) {
            telemetry.addData("Left Blue Value", CubeDetectionPipeline.getLeftBlueValue());
            telemetry.addData("Center Blue Value", CubeDetectionPipeline.getCenterBlueValue());
            telemetry.addData("Right Blue Value", CubeDetectionPipeline.getRightBlueValue());
            telemetry.addData("Position", CubeDetectionPipeline.getPosition());
            telemetry.addData("THRESHOLD", CubeDetectionPipeline.getThreshold());
            packet.put("Left Blue Value", CubeDetectionPipeline.getLeftBlueValue());
            packet.put("Center Blue Value", CubeDetectionPipeline.getCenterBlueValue());
            packet.put("Right Blue Value", CubeDetectionPipeline.getRightBlueValue());
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
        Mat blueMask = new Mat();

        private static double leftBlueValue = 0.0;
        private static double centerBlueValue = 0.0;
        private static double rightBlueValue = 0.0;

        private static double THRESHOLD = 5000.0;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

            // Blue color range
            Scalar lowerBlue = new Scalar(100, 150, 0); // Lower bound for blue in HSV format
            Scalar upperBlue = new Scalar(140, 255, 255); // Upper bound for blue in HSV format

            Mat blueMask = new Mat();

            // Apply the inRange function for the blue range
            Core.inRange(mat, lowerBlue, upperBlue, blueMask);

            // Divide the frame into three sections (left, center, right)
            Rect leftRect = new Rect(0, mat.rows() / 3, mat.cols() / 4, mat.rows() - mat.rows() / 2);
            Rect centerRect = new Rect((mat.cols() / 3) + (mat.rows() / 24), mat.rows() / 3, mat.cols() / 4, mat.rows() - mat.rows() / 2);
            Rect rightRect = new Rect((2 * mat.cols() / 3) + mat.cols() / 12, mat.rows() / 3, mat.cols() / 4, mat.rows() - mat.rows() / 2);

            Mat left = mat.submat(leftRect);
            Mat center = mat.submat(centerRect);
            Mat right = mat.submat(rightRect);

            // Calculate the sum of blue pixel values for each section
            leftBlueValue = Core.sumElems(left).val[0];
            centerBlueValue = Core.sumElems(center).val[0];
            rightBlueValue = Core.sumElems(right).val[0];

            left.release();
            center.release();
            right.release();

            // Draw rectangles around the regions
            drawBox(input, leftRect, (leftBlueValue > THRESHOLD) ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0)); // Green box if blue is high, otherwise blue
            drawBox(input, centerRect, (centerBlueValue > THRESHOLD) ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0)); // Green box if blue is high, otherwise blue
            drawBox(input, rightRect, (rightBlueValue > THRESHOLD) ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0)); // Green box if blue is high, otherwise blue

            return input;
        }

        // Get the blue pixel value for the left section
        public static double getLeftBlueValue() {
            return leftBlueValue;
        }

        // Get the blue pixel value for the center section
        public static double getCenterBlueValue() {
            return centerBlueValue;
        }

        // Get the blue pixel value for the right section
        public static double getRightBlueValue() {
            return rightBlueValue;
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

        // Get the position based on the region with the highest blue pixel value
        // Get the position based on the region with the highest blue pixel value
        public static String getPosition() {
            if (leftBlueValue > THRESHOLD && leftBlueValue > centerBlueValue && leftBlueValue > rightBlueValue) {
                return "Left";
            } else if (centerBlueValue > THRESHOLD && centerBlueValue > leftBlueValue && centerBlueValue > rightBlueValue) {
                return "Center";
            } else if (rightBlueValue > THRESHOLD && rightBlueValue > leftBlueValue && rightBlueValue > centerBlueValue) {
                return "Right";
            } else if (leftBlueValue + centerBlueValue + rightBlueValue > THRESHOLD) {
                // If the sum of blue pixel values in all sections is above the threshold, but none of the individual sections is high enough
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