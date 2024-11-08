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

@Autonomous(name = "Test_Actions")

public class Test_Actions extends LinearOpMode {

    private Servo Claw1;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        Pose2d beginPose = new Pose2d(-9, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .afterTime(1, Claw(true))
                        .afterTime(3, Claw(false))

                        .build());
    }
//METHODS-------------------------------------------------------------------------------------------------------------------

    private void initHardware() {
        //Motors
        Claw1 = hardwareMap.get(Servo.class, "Claw1");
        Claw1.setPosition(0);

    }

    private Action Claw(boolean sample) {
        if (sample) {
            Claw1.setPosition(1);
        } else {
            Claw1.setPosition(0);
        }
        return null;
    }
}