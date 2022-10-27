package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Autonomous
public class Vroom extends LinearOpMode {
    private DcMotorEx rightSlide, leftSlide;

    @Override
        public void runOpMode() {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightslide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftslide");

            Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(15)
                    .build();

            Trajectory mytraj2 = drive.trajectoryBuilder(myTrajectory.end())
                            .forward(12).build();



            waitForStart();

            if (isStopRequested()) return;

            drive.followTrajectory(myTrajectory);
            drive.followTrajectory(mytraj2);
            raise(1,800);
            raise(-1,700);

        }
        private void raise(double power, long time){
            rightSlide.setPower(power);
            leftSlide.setPower(-1*power);
            sleep(time);

        }

    }


