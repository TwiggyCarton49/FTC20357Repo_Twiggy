package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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
    public int target = -400;
    public class Claw {
        private Servo claw;

        public Claw (HardwareMap hardwareMap){
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class closeClaw implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket packet){
                claw.setPosition(1); //What our close position is
                sleep(3000);
                return false;
            }
        }
        public class openClaw implements Action{
            @Override
            public boolean run (@NonNull TelemetryPacket packet){
                claw.setPosition(0); //What our open position is
                return false;
            }
        }
        public Action Open(){
            return new Claw.openClaw();
        }
        public Action Close(){
            return new Claw.closeClaw();
        }
    }

    public DcMotorEx joint1,joint2,Slide;

    public class Arm {

        public Arm(HardwareMap hardwareMap) {
            joint1 = hardwareMap.get(DcMotorEx.class, "joint1");
            joint2 = hardwareMap.get(DcMotorEx.class, "joint2");
            Slide = hardwareMap.get(DcMotorEx.class, "extend");
        }

        public class Controller{
            PIDController controller;
            public double p,i,d;
            private double f;
            //public double p1 = 0.004=, i1 = 0.001, d1 = 0.0005;

            //private double f1 = 0.185;

            private final double ticks_in_degrees = 1440 / 180;
            public Controller(int p, int i, int d, int f){
                this.p = p;
                this.i = i;
                this.d = d;
                this.f = f;
            }
            public double calc(int target){
                controller = new PIDController(this.p, this.i, this.d);
                controller.setPID(this.p,this.i, this.d);
                int armPos = joint1.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * this.f;

                double power = pid + ff;
                return power;
            }
        }

        public class PIDAction implements Action{



            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(24, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);

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

        if(!isStopRequested()) return;

        Action action1 = sample.build();
        Action action2 = traj.build();
//Parallel Actions?
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .stopAndAdd(claw.Open())
                        .stopAndAdd(arm.joint1up(-500))
                        .build()
                );
        Actions.runBlocking(
                new ParallelAction(

                )
        );


    }
}




