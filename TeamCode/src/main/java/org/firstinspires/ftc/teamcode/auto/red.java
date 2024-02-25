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

@Autonomous(name="Cube Detection Red")
public class red extends LinearOpMode {
    OpenCvCamera webcam;
    CubeDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        setupcam();

        waitForStart();
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        while (opModeIsActive()) {
            telemetry.addData("Analysis", CubeDetectionPipeline.getAnalysis());
            telemetry.addData("Position", CubeDetectionPipeline.getPosition());
            packet.put("Analysis", pipeline.getAnalysis());
            packet.put("Position", pipeline.getPosition());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
            sleep(100);
        }
    }
    public void setupcam(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CubeDetectionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }
    public static class CubeDetectionPipeline extends OpenCvPipeline {
        Mat mat = new Mat();
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        public enum Position {
            LEFT,
            CENTER,
            RIGHT,
            UNKNOWN
        }
        private static Position position = Position.UNKNOWN;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Scalar lowerRed1 = new Scalar(0, 150, 100); // Lower range of red (hue 0-10)
            Scalar upperRed1 = new Scalar(10, 255, 255);
            Scalar lowerRed2 = new Scalar(160, 150, 100); // Upper range of red (hue 160-179)
            Scalar upperRed2 = new Scalar(179, 255, 255);

            Core.inRange(mat, lowerRed1, upperRed1, mask1);
            Core.inRange(mat, lowerRed2, upperRed2, mask2);
            Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mat); // Combine both red masks

            // Define regions of interest (ROIs) for left, center, and right positions
            Rect leftRect = new Rect(0, 0, mat.cols() / 3, mat.rows());
            Rect centerRect = new Rect(mat.cols() / 3, 0, mat.cols() / 3, mat.rows());
            Rect rightRect = new Rect(2 * mat.cols() / 3, 0, mat.cols() / 3, mat.rows());

            Mat left = mat.submat(leftRect);
            Mat center = mat.submat(centerRect);
            Mat right = mat.submat(rightRect);

            double leftValue = Core.sumElems(left).val[0];
            double centerValue = Core.sumElems(center).val[0];
            double rightValue = Core.sumElems(right).val[0];

            left.release();
            center.release();
            right.release();

            if (leftValue > centerValue && leftValue > rightValue) {
                position = Position.LEFT;
            } else if (centerValue > leftValue && centerValue > rightValue) {
                position = Position.CENTER;
            } else if (rightValue > leftValue && rightValue > centerValue) {
                position = Position.RIGHT;
            } else {
                position = Position.UNKNOWN;
            }

            return input;
        }

        public static Position getPosition() {
            return position;
        }

        public static String getAnalysis() {
            return position.toString();
        }
    }
}