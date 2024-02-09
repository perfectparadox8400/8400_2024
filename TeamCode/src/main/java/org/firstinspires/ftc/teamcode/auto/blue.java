import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.DetectionPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="Cube Detection Blue")
public class blue extends LinearOpMode {
    OpenCvCamera webcam;
    CubeDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CubeDetectionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.getPosition());
            telemetry.update();

            sleep(100);
        }
    }

    public static class CubeDetectionPipeline extends OpenCvPipeline {
        Mat mat = new Mat();
        public enum Position {
            LEFT,
            CENTER,
            RIGHT,
            UNKNOWN
        }
        private Position position = Position.UNKNOWN;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Scalar lowerBlue = new Scalar(100, 150, 100); // Example HSV range for blue
            Scalar upperBlue = new Scalar(140, 255, 255);
            Core.inRange(mat, lowerBlue, upperBlue, mat);

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

        public Position getPosition() {
            return position;
        }

        public String getAnalysis() {
            return position.toString();
        }
    }
}
