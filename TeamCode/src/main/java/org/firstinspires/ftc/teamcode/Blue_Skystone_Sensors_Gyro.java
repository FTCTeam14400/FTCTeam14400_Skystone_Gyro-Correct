package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.util.Range.clip;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *
 */
@Autonomous(name= "Red Skystone Gyro", group="Sky autonomous")
//@Disabled
public class Blue_Skystone_Sensors_Gyro extends LinearOpMode {
    HardwareMechanum robot       = new HardwareMechanum(); // use the class created to define a Pushbot's hardware
    BNO055IMU imu;
    Orientation angles;
    private ElapsedTime runtime = new ElapsedTime();




    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = 1f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 3f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 3f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 3f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor
    private AndroidTextToSpeech androidTextToSpeech;

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;


    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: Neverest 40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = .77 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static double           status                  = 0;
    private double rotadjust = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Hardware Init Status: ", "Initializing");
        telemetry.addData("Camera Init Status: ", "Waiting...");
        telemetry.addData("Speech Init Status: ", "Waiting...");
        telemetry.update();
        sleep(500);
        robot.init(hardwareMap);
        telemetry.addData("Hardware Init Status: ", "Complete");
        telemetry.addData("Camera Init Status: ", "Initializing");
        telemetry.update();
        androidTextToSpeech = new AndroidTextToSpeech();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        telemetry.addData("Camera Init Status: ", "Complete");
        telemetry.addData("Speech Init Status: ", "Initializing");
        telemetry.update();
        androidTextToSpeech.initialize();
        androidTextToSpeech.setLanguageAndCountry("en", "US");
        sleep(100);
        telemetry.addData("Speech Init Status: ", "Complete");
        telemetry.update();
        // Wait for user to push start button.
        robot.BlinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Stone Detection Values", valLeft+"   "+valMid+"   "+valRight);

            if (valLeft == 0) {
                strafeLeft(.8,12 ,2);
                robot.Rotate.setPower(-1);
                sleep(400);
                robot.Rotate.setPower(0);
                robot.spool.setPower(-.5);
                sleep(100);
                robot.spool.setPower(0);
                robot.claw.setPosition(.5);
                sleep(800);
                forward(.7, 20, 2);
                sleep(800);
                robot.claw.setPosition(.2);
                backward(.7, 6, 2);
                turnLeft(1, 22, 3);
                forward(.7, 40, 2);
                robot.claw.setPosition(.5);
                sleep(800);
                backward(.7, 50, 2);
                robot.claw.setPosition(.8);
                sleep(800);
                robot.spool.setPower(.5);
                sleep(100);
                robot.spool.setPower(0);
                robot.Rotate.setPower(1);
                sleep(400);
                robot.Rotate.setPower(0);
                sleep(30000);

            }
            else if (valMid == 0){

            }
            else if (valRight == 0) {

            }






            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        }

    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
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
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
    /*
    public void sensorStrafe2(double spd, double distance, double desired) {
        while (  opModeIsActive()){

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
           rotadjust = (angles.firstAngle - desired) * .01;
           telemetry.addData("Gyro", angles.firstAngle);
           rotadjust = clip(.2,.2,rotadjust);
            robot.leftBack.setPower(spd + rotadjust);
            robot.rightBack.setPower(-spd - rotadjust);
            robot.leftFront.setPower(-spd + rotadjust);
            robot.rightFront.setPower(spd - rotadjust) ;
            telemetry.addData("Distance", robot.stoneDistance.getDistance(DistanceUnit.INCH));
        }
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
    }

*/
    /*

     */


    public void forward(double spd, double fwd,double timeoutS) {
        encoderDrive(spd, fwd, fwd, fwd, fwd, timeoutS);
        sleep(200);
    }
    public void backward(double spd, double back,double timeoutS) {
        encoderDrive(spd, -back, -back, -back, -back, timeoutS);
        sleep(200);
    }
    public void strafeRight(double spd, double strafe, double timeoutS){
        encoderDrive(spd, -strafe, strafe, strafe, -strafe, timeoutS);

    }
    public void strafeLeft(double spd, double strafe, double timeoutS){
        encoderDrive(spd, -strafe, strafe, -strafe, strafe, timeoutS);

    }
    public void turnRight(double spd, double turn, double timeoutS) {
        encoderDrive(spd, turn, -turn, turn, -turn,  timeoutS);
        sleep(200);
    }
    public void turnLeft(double spd, double turn,double timeoutS ) {
        encoderDrive(spd, turn, -turn, -turn, turn,  timeoutS);
        sleep(200);
    }
    public void encoderDrive(double speed, double BackleftInches, double BackrightInches, double FrontleftInches, double FrontrightInches,
                             double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robot.leftBack.getCurrentPosition() + (int)(BackleftInches * COUNTS_PER_INCH) ;
            newBackRightTarget = robot.rightBack.getCurrentPosition() + (int)(BackrightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.leftFront.getCurrentPosition() + (int)(FrontleftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.rightFront.getCurrentPosition() + (int)(FrontrightInches * COUNTS_PER_INCH);
            robot.leftBack.setTargetPosition(newBackLeftTarget);
            robot.rightBack.setTargetPosition(newBackRightTarget);
            robot.leftFront.setTargetPosition(newFrontLeftTarget);
            robot.rightFront.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftBack.setPower(Math.abs(speed));
            robot.rightBack.setPower(Math.abs(speed));
            robot.leftFront.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftBack.isBusy() && robot.rightBack.isBusy() && robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftBack.getCurrentPosition(),
                        robot.rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


}
