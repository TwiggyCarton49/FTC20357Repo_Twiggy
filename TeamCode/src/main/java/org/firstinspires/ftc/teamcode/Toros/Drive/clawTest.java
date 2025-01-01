package org.firstinspires.ftc.teamcode.Toros.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Toros.Util.BatteryClass;


@TeleOp(name = "clawTest")
public class clawTest extends LinearOpMode {
    private Servo fingers;
    private Servo wrist;
    private Servo elbow;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();


        telemetry.addData("motor power", fingers.getPosition());
        initTelemetry();
        telemetry.update();
    }


    private void initHardware() {
        fingers = hardwareMap.get(Servo.class, "fingers");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");

        fingers.setPosition(1);
        wrist.setPosition(1);
        elbow.setPosition(1);

    }

    private void initTelemetry() {

        BatteryClass battery = new BatteryClass(hardwareMap);
        telemetry.addData("Battery", battery.getBatteryPercent());
        telemetry.addData("fingers pos", fingers.getPosition());
        telemetry.addData("wrist pos", wrist.getPosition());
        telemetry.addData("elbow pos", elbow.getPosition());

        telemetry.update();
    }

    private void claw() {
        if (gamepad2.left_bumper) {
            fingers.setPosition(0);
        } else if (gamepad2.right_bumper) {
            fingers.setPosition(1);
        }



    }
}
