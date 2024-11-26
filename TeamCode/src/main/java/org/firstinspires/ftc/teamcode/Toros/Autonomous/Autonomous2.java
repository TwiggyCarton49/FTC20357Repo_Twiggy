package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

public class Autonomous2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(11,6,Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-9,-35))
                //Place Specimen
                .waitSeconds(2)
                .strafeTo(new Vector2d(-48,-40))
                //Pick up sample 1
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))
                //Place sample 1
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-58,-40), Math.toRadians(90))
                //Pick up sample 2
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))
                //Place sample 2
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-54,-25), Math.toRadians(180))
                //Pick up Sample 3
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))
                //Place Sample 3
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-23, -12), Math.toRadians(0));

        tab1.build();



    }
    private void arm(){

    }
    private void claw(){

    }
}
