package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Vroom extends LinearOpMode {
    private DcMotorEx rightSlide, leftSlide;
    private Servo claw;

    @Override
        public void runOpMode() {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

         //Intializies the hardware
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightslide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftslide");
<<<<<<< HEAD
        claw = hardwareMap.get(Servo.class, "claw");
=======
        claw = hardwareMap.get(servo.class, "claw");
        /*
        Creates a 2d array to map out the junctions and poles. Each array is a row and each element
        is a col. The values for each Vector2d in the array represents the vector2d postion of a pole
        or junction
         */

        Vector2d[][] poles = new Vector2d[5][5];

        for(int i = 0; i < 5; i++){
            int yStart = 72;
            yStart -= 18;
            for(int j = 0; i < 5; i++){
                int xStart = -72;
                xStart += 18;
                poles[i][j] = new Vector2d(xStart, yStart);
            }
        }
>>>>>>> 341ca2b0f2262992e9678993b243fc6915f38fa7

        //Creates first
            Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(16)
                    .build();

            Trajectory mytraj2 = drive.trajectoryBuilder(myTrajectory.end())
                            .forward(12).build();



            waitForStart();

            if (isStopRequested()) return;

            claw.setPosition(1);
            drive.followTrajectory(myTrajectory);
            drive.followTrajectory(mytraj2);
            raise(1,900);
            sleep(4000);
            claw.setPosition(0);
            lower(1,700);

        }
        private void raise(double power, long time){
            rightSlide.setPower(power);
            leftSlide.setPower(-1*power);
            sleep(time);

        }
        private void lower(double power, long time){
            rightSlide.setPower(-1*power);
            leftSlide.setPower(power);
            sleep(time);

            }

    }


