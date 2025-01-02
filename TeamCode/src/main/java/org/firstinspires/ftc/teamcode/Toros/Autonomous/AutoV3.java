package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

@Autonomous
public class AutoV3 extends LinearOpMode {

    public class Claw {
        private Servo fingers, wrist, elbow;

        public Claw(HardwareMap hardwareMap) {
            fingers = hardwareMap.get(Servo.class, "fingers");
            wrist = hardwareMap.get(Servo.class, "wrist");
            elbow = hardwareMap.get(Servo.class, "elbow");
        }

        public class openFingers implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                fingers.setPosition(0);
                return false;
            }
        }
        public class closeFingers implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                fingers.setPosition(1);
                return false;
            }


        }

    }


        public void runOpMode() throws InterruptedException {
            Pose2d initialPose = new Pose2d(24, -61, Math.toRadians(90));
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

            TrajectoryActionBuilder sample = drive.actionBuilder(initialPose)
                    .strafeTo(new Vector2d(8, -33))
                    .waitSeconds(1);
            TrajectoryActionBuilder traj = drive.actionBuilder(drive.pose)
                    .lineToY(-40)
                    .setTangent(Math.toRadians(350))
                    .splineToLinearHeading(new Pose2d(48, -10, Math.toRadians(270)), Math.toRadians(270))
                    .strafeTo(new Vector2d(48, -52))
                    .strafeTo(new Vector2d(48, -15))
                    .strafeTo(new Vector2d(58, -15))
                    .strafeTo(new Vector2d(58, -52))
                    .strafeTo(new Vector2d(58, -15))
                    .strafeTo(new Vector2d(64, -15))
                    .strafeTo(new Vector2d(64, -52));
            Action trjEnd = traj.endTrajectory().fresh()
                    .build();
//        TrajectoryActionBuilder firstTraj = drive.actionBuilder(drive.pose)

            waitForStart();

            if (!isStopRequested()) return;

            Action action1 = sample.build();
            Action action2 = traj.build();
//Parallel Actions?
            Actions.runBlocking(
                    new SequentialAction(

                    )

            );


        }
    }





