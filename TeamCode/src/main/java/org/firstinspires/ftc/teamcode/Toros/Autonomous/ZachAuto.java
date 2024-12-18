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
public class ZachAuto extends LinearOpMode {
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



    public class Arm {
        public DcMotorEx SlideLeft,SlideRight,MiddleSlide;

        public Arm(HardwareMap hardwareMap) {
            SlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
            SlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");
            MiddleSlide = hardwareMap.get(DcMotorEx.class, "MiddleSlide");
        }

        public class Controller{
            PIDController controller;
            public double p,i,d;
            private double f;
            //public double p1 = 0.004=, i1 = 0.001, d1 = 0.0005;

            //private double f1 = 0.185;

            private final double ticks_in_degrees = 1440 / 180;
            public Controller(double p, double i, double d, double f){
                this.p = p;
                this.i = i;
                this.d = d;
                this.f = f;
            }
            public double calc(int target){
                controller = new PIDController(this.p, this.i, this.d);
                controller.setPID(this.p,this.i, this.d);
                int armPos = SlideLeft.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * this.f;

                double power = pid + ff;
                return power;
            }
        }
        public class joint1Up implements InstantFunction{
            int t;
            Controller controller = new Controller(0.004,0.001,0.0004,0.185);
            public joint1Up(int target){
                this.t = target;
            }

            @Override
            public void run() {
                SlideLeft.setPower(controller.calc(this.t));
            }
        }
        public InstantFunction joint1up(int target){
            return new joint1Up(target);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(24, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        TrajectoryActionBuilder sample = drive.actionBuilder(initialPose);

        TrajectoryActionBuilder traj = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(8, -36))
                //Hang Spec 1
                .strafeTo(new Vector2d(8,-40))
                .strafeTo(new Vector2d(30, -38))
                //.strafeTo(new Vector2d(40, -12))
                .splineToConstantHeading(new Vector2d(41,-12), Math.toRadians(0))
                .strafeTo(new Vector2d(46, -55))//push spec 3
                //pick up spec 2
                .strafeTo(new Vector2d(0, -35))
                //Hang spec 2
                .strafeTo(new Vector2d(20,-38))
                .splineToConstantHeading(new Vector2d(55,-14), Math.toRadians(0))
                .strafeTo(new Vector2d(55, -55))//push spec 4
                //pick up spec 3
                .strafeTo(new Vector2d(5, -35))
                //Hang spec 3
                .strafeTo(new Vector2d(20,-38))
                .splineToConstantHeading(new Vector2d(62,-14), Math.toRadians(0))
                .strafeTo(new Vector2d(62, -55))//push spec 5
                //Pick up spec 4
                .strafeTo(new Vector2d(0, -35))
                //Hang spec 4

                //END 4 sample Auto!!!!!!!

//--------------------------------------------------------------------------------------------------
                //Optional 5 Spec Auto
                .strafeTo(new Vector2d(40, -50))
                .strafeTo(new Vector2d(40, -55))
                //pick up spec 5
                .strafeTo(new Vector2d(-2, -35))
                //Hang spec 5
//--------------------------------------------------------------------------------------------------

                .strafeTo(new Vector2d(39, -58));//park

        Action trjEnd = traj.endTrajectory().fresh()
                .build();
//        TrajectoryActionBuilder firstTraj = drive.actionBuilder(drive.pose)

        waitForStart();

        if(!isStopRequested()) return;

        Action action1 = sample.build();
        Action action2 = traj.build();
//Parallel Actions?
        Actions.runBlocking(
                new ParallelAction(
                        new InstantAction(arm.joint1up(-500)),
                        new SequentialAction(

                        )
                )
        );


    }
}





