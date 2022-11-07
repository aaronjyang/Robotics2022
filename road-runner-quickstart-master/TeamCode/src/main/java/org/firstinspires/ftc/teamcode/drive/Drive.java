package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {

    private Servo claw;
    private DcMotor frontleft;
    private DcMotor backleft;
    private DcMotor frontright;
    private DcMotor backright;
    private DcMotor leftslide;
    private DcMotor rightslide;
    private int slidePos = 0;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int Timing;
        boolean control;
        float y;
        float x;
        float rx;
        double denom;

        claw = hardwareMap.get(Servo.class, "claw");
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        backright = hardwareMap.get(DcMotor.class, "back right");
        leftslide = hardwareMap.get(DcMotor.class, "leftslide");
        rightslide = hardwareMap.get(DcMotor.class, "rightslide");
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        // Put initialization blocks here.
        claw.setPosition(1);
        control = true;
        Timing = 0;
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Read inverse IMU heading, as the IMU heading is CW positive
                double botHeading = -imu.getAngularOrientation().firstAngle;

                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;


                double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

                denom = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(rx), Math.abs(x), Math.abs(y))), 1));
                if (gamepad1.right_bumper) {
                    denom = denom * 3;
                    telemetry.addData("thing", 3);
                }
                frontleft.setPower((rotY + rotX + rx) / denom);
                backleft.setPower(((rotY - rotX) + rx) / denom);
                frontright.setPower(((rotY - rotX) - rx) / denom);
                backright.setPower(((rotY + rotX) - rx) / denom);
                telemetry.update();
                // Put loop blocks here.
                frontleft.setPower(0);
                backleft.setPower(0);
                backright.setPower(0);
                frontright.setPower(0);
                if (gamepad1.b) {
                    claw.setPosition(0);
                } else if (gamepad1.x) {
                    claw.setPosition(1);
                }
                if (gamepad1.y) {
                    leftslide.setPower(-0.9);
                    rightslide.setPower(1);
                    slidePos++;
                } else if (gamepad1.a) {
                  if(slidePos>-1){
                      leftslide.setPower(1);
                      rightslide.setPower(-1);
                      slidePos--;
                  }

                }
                if (Timing < 0) {
                    Timing = 0;
                }
                telemetry.addData("thing", 0);
                telemetry.addData("slide position", slidePos);
                telemetry.update();
                leftslide.setPower(0);
                rightslide.setPower(0);
            }
        }
    }
}
