package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;


public class Autonomous extends LinearOpMode {
    public class elevator{
        public elevator(HardwareMap hardwareMap){
            Elevator = hardwareMap.get(DcMotorEx.class,"elev");
        }
    }
    public class LiftUp implements Action {
        private boolean inited = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!inited){
                Elevator.setPower(0.5);
                inited = true;
            }

            double pos = Elevator.getCurrentPosition();
            if(pos < 3000){
                return true;
            }
            else {
                Elevator.setPower(0);
                return false;
            }
        }

    }
    public class Liftdown implements Action {
        private boolean inited = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!inited){
                Elevator.setPower(-0.5);
                inited = true;
            }

            double pos = Elevator.getCurrentPosition();
            if(pos < 3000){
                return true;
            }
            else {
                Elevator.setPower(0);
                return false;
            }
        }

    }
    public Action liftUp(){

        return new LiftUp();
    }
    public Action liftDown(){
        return new Liftdown();
    }
    private PIDController controller;

    private Servo Claw1;

    public static double p1 = 0.03, i1 = 0, d1 = -0.0001;

    public static double f1 = -0.05;

    public static int target1 = 0;

    private final double ticks_in_degrees = 1440/180;
    private DcMotorEx ArmPivot1;
    private DcMotorEx ArmPivot2;
    private DcMotorEx Elevator;

    @Override
    public void runOpMode() throws InterruptedException {
        ArmPivot1 = hardwareMap.get(DcMotorEx.class, "Pivot1");
        ArmPivot2 = hardwareMap.get(DcMotorEx.class, "Pivot2");

        Claw1 = hardwareMap.get(Servo.class, "Claw1");
        if (opModeIsActive()) {
            Pose2d initialPose = new Pose2d(11, 6, Math.toRadians(0));
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
            TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                    .strafeTo(new Vector2d(0, -52))
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(8, -33))
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(48, -37))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(51, -62), Math.toRadians(270))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(58, -37), Math.toRadians(90))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(53, -62), Math.toRadians(270))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(58, -25), Math.toRadians(0))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(60, -62), Math.toRadians(270));

            Action trajChosen = tab1.build();
            Actions.runBlocking(
                    new SequentialAction(
                         trajChosen,
                            liftUp(),
                            liftDown()
                    )

            );

        }
    }
    private void arm(int target) {
        controller = new PIDController(p1, i1, d1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            controller.setPID(p1, i1, d1);
            int armPos = ArmPivot1.getCurrentPosition();
            int armPos2 = ArmPivot2.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target1 / ticks_in_degrees)) * f1;

            double power = pid + ff;

            ArmPivot1.setPower(power);
            ArmPivot2.setPower(power);

            telemetry.addData("Pos", armPos);
            telemetry.addData("Pos2", armPos2);
            telemetry.addData("Target", target1);

            telemetry.update();
        }
        private void claw(double x){
            Claw1.setPosition(x);
        }

    }
