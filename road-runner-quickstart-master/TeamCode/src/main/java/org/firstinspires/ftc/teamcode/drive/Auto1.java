/*
 * Copyright (c) 2021 OpenFTC Team
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

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;

@Autonomous
public class Auto1 extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    int time2;


    static final double FEET_PER_METER = 3.28084;

    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor frontleft;
    private DcMotor frontright;
    private Servo claw;
    private DcMotor leftslide;
    private DcMotor rightslide;


    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    @Override
    public void runOpMode()
    {
        backleft = hardwareMap.get(DcMotor.class, "back left");
        backright = hardwareMap.get(DcMotor.class, "back right");
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        claw = hardwareMap.get(Servo.class, "claw");
        leftslide = hardwareMap.get(DcMotor.class, "leftslide");
        rightslide = hardwareMap.get(DcMotor.class, "rightslide");


        int detectedTag = -1;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        if(isStopRequested()){
            return;
        }


        waitForStart();
        time2 = 750;
        claw.setPosition(1);
        pause();
        claw.setPosition(0);
        pause();
        StrafeRight(1);
        Raise(800);
        forwardRicky(200);
        pause();
        Lower(110);
        pause();
        claw.setPosition(1);
        pause();
        Raise(75);
        pause();
        Backward(500);
        StrafeLeft(100);
        Lower(700);
        Backward(100);
        pause();

        telemetry.setMsTransmissionInterval(50);

        while (opModeIsActive()) {
            if (detectedTag < 0) {
                // Calling getDetectionsUpdate() will only return an object if there was a new frame
                // processed since the last time we called it. Otherwise, it will return null. This
                // enables us to only run logic when there has been a new frame, as opposed to the
                // getLatestDetections() method which will always return an object.
                ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

                // If there's been a new frame...
                if (detections != null) {
                    // If we don't see any tags
                    if (detections.size() == 0) {
                        numFramesWithoutDetection++;

                        // If we haven't seen a tag for a few frames, lower the decimation
                        // so we can hopefully pick one up if we're e.g. far back
                        if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                            aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                        }
                    }
                    // We do see tags!
                    else {
                        numFramesWithoutDetection = 0;

                        // If the target is within 1 meter, turn on high decimation to
                        // increase the frame rate
                        if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                            aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                        }

                        for (AprilTagDetection detection : detections) {
                            detectedTag = detection.id;
                            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                        }
                    }

                    telemetry.update();
                }

                sleep(20);
            }
            if (detectedTag == 1){
                telemetry.addLine(String.format("\nDetected tag ID=%d", detectedTag));
                forward();
                sleep(1100);
                stop2();
                break;
            } else if (detectedTag == 0){
                telemetry.addLine(String.format("\nDetected tag ID=%d", detectedTag));
                forward();
                sleep(1000);
                left();
                sleep(1500);
                stop2();
                break;

            } else if (detectedTag == 2){
                telemetry.addLine(String.format("\nDetected tag ID=%d", detectedTag));
                forward();
                sleep(1100);
                right();
                sleep(1300);
                stop2();
                break;
            } else {
                telemetry.addLine(String.format("\nDetected tag ID=%d", detectedTag));

            }
        }
    }
    private void stop2() {
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }
    private void forward() {
        backleft.setPower(-1);
        backright.setPower(1);
        frontleft.setPower(-1);
        frontright.setPower(1);
    }
    private void forwardRicky(int time1) {
        backleft.setPower(-1);
        backright.setPower(1);
        frontleft.setPower(-1);
        frontright.setPower(1);
        sleep(time1);
    }

    private void left(){
        backleft.setPower(-0.9);
        backright.setPower(-1);
        frontleft.setPower(1);
        frontright.setPower(1);
    }
    private void right(){
        backleft.setPower(1);
        backright.setPower(1);
        frontleft.setPower(-1);
        frontright.setPower(-1);

    }

    private void Backward(int time1) {
        backleft.setPower(1);
        backright.setPower(-1);
        frontleft.setPower(1);
        frontright.setPower(-1);
        sleep(time1);
    }


    /**
     * Describe this function...
     */
    private void turn_left() {
        backleft.setPower(1);
        backright.setPower(1);
        frontleft.setPower(1);
        frontright.setPower(1);
        sleep(time2);
    }

    /**
     * Describe this function...
     */
    private void Open() {
        claw.setPosition(0);
    }

    /**
     * Describe this function...
     */
    private void pause() {
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
        leftslide.setPower(0);
        rightslide.setPower(0);
        sleep(time2);
    }

    /**
     * Describe this function...
     */
    private void StrafeRight(int time1) {
        backleft.setPower(1);
        backright.setPower(1);
        frontleft.setPower(-1);
        frontright.setPower(-1);
        sleep(time1);
    }

    private void StrafeLeft(int time1) {
        backleft.setPower(-1);
        backright.setPower(-1);
        frontleft.setPower(1);
        frontright.setPower(1);
        sleep(time1);
    }

    /**
     * Describe this function...
     */
    private void Lower(int time1) {
        leftslide.setPower(0.9);
        rightslide.setPower(-0.8);
        sleep(time1);
    }

    /**
     * Describe this function...
     */
    private void Raise(int time1) {
        leftslide.setPower(-1);
        rightslide.setPower(1);
        sleep(time1);
    }

}