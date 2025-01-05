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
import com.qualcomm.robotcore.hardware.DcMotor;
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
                fingers.setPosition(0);
                return false;
            }


        }
        public class openWrist implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.setPosition(1);
                return false;
            }


        }
        public class closeWrist implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.setPosition(0);
                return false;
            }


        }
        public class closeElbow implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                elbow.setPosition(1);
                return false;
            }


        }
        public class openElbow implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                elbow.setPosition(0);
                return false;
            }


        }
        public Action fingersOpen() { return new openFingers();}
        public Action fingersClose(){return new closeFingers();}
        public Action wristClose(){return new closeWrist();}
        public Action wristOpen(){return new openWrist();}
        public Action elbowOpen(){return new openElbow();}
        public Action elbowClose(){return new closeElbow();}
    }
    public class pivot{
        private DcMotorEx pivot;
        public pivot(HardwareMap hardwareMap){
            pivot = hardwareMap.get(DcMotorEx.class,"pivot");
        }

        public class runPivot implements Action{

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivot.setPower(1);
                return true;
            }

        }
        public Action pivotRun(){return new runPivot();}
    }
    public class lift{
        int target1 = 0;

        private final DcMotorEx slideLeft, slideRight;
        public lift(HardwareMap hardwareMap){
            slideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
            slideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        }
        public class LiftPID implements Action{

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                PIDController controller;
                double p1 = 0, i1 = 0, d1 = 0;

                double f1 = 0;


                controller = new PIDController(p1,i1,d1);
                double ticks_in_degrees = 1440/180;

                controller.setPID(p1,i1,d1);
                int armPos = slideLeft.getCurrentPosition();
                double pid = controller.calculate(armPos, target1);
                double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

                double power = pid + ff;

                slideLeft.setPower(power);
                slideRight.setPower(power);
                return true;
            }
        }
        public class ChangeLiftTarget implements Action{
            int t;
            public ChangeLiftTarget(int target){
                this.t = target;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target1 = this.t;
                return false;
            }
        }
        public Action runPID(){
            return new LiftPID();
        }
        public Action changeTarget(int target){
            return new ChangeLiftTarget(target);
        }
    }


        public void runOpMode() throws InterruptedException {
            Pose2d initialPose = new Pose2d(24, -61, Math.toRadians(90));
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
            lift lift = new lift(hardwareMap);
            pivot pivot = new pivot(hardwareMap);
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
                    new ParallelAction(
                            lift.runPID()
                            //lift.changeTarget(270),
                            //pivot.pivotRun()
                    )

            );


        }
    }





