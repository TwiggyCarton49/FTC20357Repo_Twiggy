package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import java.util.Objects;

@Autonomous(name = "Left_Side_Auto")

public class Left_Auto extends LinearOpMode {
    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackRightMotor;
    private DcMotor ArmPivot1;
    private DcMotor Elevator;
    private DcMotor Elevator2;
    private Servo Claw1;



    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        Pose2d beginPose = new Pose2d(-9, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        .afterTime(0, Arm(50, 0.5, 80, 0.4))//Extend at start
                        .strafeTo(new Vector2d(-9,-35))//Move to Specimen

                        .afterTime(1, Arm(-10, -0.2, 0, 0))//Place Specimen
                        .afterTime(1.1, Claw(true))//release Specimen

                        .afterTime(2, Arm(-40, -0.3, -80, -0.2))//Lower arm
                        .strafeTo(new Vector2d(-48,-40))//Move to Sample 1

                        .afterTime(3, Claw(false))//Grab sample 1
                        .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))//Move to Basket

                        .afterTime(3.1, Arm(100, 1.0, 90, 0.5))//Raise arm
                        .afterTime(3.2, Claw(true))//Score Sample 1

                        .afterTime(4, Arm(-100, -0.8, -90, -0.5))//Lower Arm
                        .strafeToLinearHeading(new Vector2d(-58,-40), Math.toRadians(90))//Move to Sample 2

                        .afterTime(5, Claw(false))//Grab Sample 2
                        .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))//Move to Basket

                        .afterTime(5.1, Arm(100, 1.0, 90, 0.5))//Raise arm
                        .afterTime(5.2, Claw(true))//Score Sample 1

                        .afterTime(6, Arm(-100, -0.8, -90, -0.5))//Lower Arm
                        .strafeToLinearHeading(new Vector2d(-54,-25), Math.toRadians(180))//Move to sample 3

                        .afterTime(7, Claw(false))//Grab Sample 3
                        .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))//Move to Basket
                        .afterTime(7.1, Arm(100, 1.0, 90, 0.5))//Raise arm
                        .afterTime(7.2, Claw(true))//Score Sample 3

                        .afterTime(8, Arm(-60, -0.5, -45, -0.3))//Lower Arm to bar
                        .strafeToLinearHeading(new Vector2d(-23, -12), Math.toRadians(0))//Park

                        .build());
    }

//METHODS-------------------------------------------------------------------------------------------------------------------

    private void initHardware() {
        //Motors
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");

        Elevator = hardwareMap.get(DcMotor.class, "elevator");
        Elevator2 = hardwareMap.get(DcMotor.class, "elevator2");

        ArmPivot1 = hardwareMap.get(DcMotor.class,"Pivot1");

        Claw1 = hardwareMap.get(Servo.class, "Claw1");

        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Elevator2.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmPivot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Claw1.setPosition(0);


    }

    private Action Arm(int extend,double eSpeed, int pos, double rSpeed) {

        Elevator.setPower(eSpeed);
        Elevator2.setPower(eSpeed);
        ArmPivot1.setPower(rSpeed);

        ArmPivot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmPivot1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmPivot1.setTargetPosition(ArmPivot1.getCurrentPosition() + pos);

        Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Elevator.setTargetPosition(Elevator.getCurrentPosition() + extend);

        Elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Elevator2.setTargetPosition(Elevator2.getCurrentPosition() + extend);

        //Add zero power for rest pos? BRAKE OR FLOAT??????
        return Objects::nonNull;
    }

    private Action Claw(boolean sample) {
        if (sample) {
            Claw1.setPosition(1);
        } else {
            Claw1.setPosition(0);
        }
        return Objects::nonNull;
    }
}