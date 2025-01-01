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

    public class Arm {
        public DcMotorEx slideLeft,slideRight,pivot;

        public Arm(HardwareMap hardwareMap) {
            slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
            slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
            //pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        }

        public class Controller{

            }
            public double calc(int target){


                return slideLeft.getCurrentPosition();
            }
        }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}




