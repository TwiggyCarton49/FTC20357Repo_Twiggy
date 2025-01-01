package org.firstinspires.ftc.teamcode.Toros.Util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp
public class ArmPID extends LinearOpMode {
    private PIDController controller;
    private PIDController controller2;



    public static double p1 = 0.0009, i1 = 0.00, d1 = 0.000;

    public static double f1 = 0.00;

    public static int target1 = 425;
    public static double p2 = 0.04, i2 = 0.0, d2 = 0.0001;

    public static double f2 = 0.003;

    public static int target2 = 250;

    private final double ticks_in_degrees = 1440/180;
    private DcMotorEx pivot;

    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(p1,i1,d1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        pivot = hardwareMap.get(DcMotorEx.class,"pivot");
        telemetry.addData("Pos", pivot.getCurrentPosition());
        telemetry.addData("Target", target1);
        waitForStart();
        while (opModeIsActive()){
            controller.setPID(p1,i1,d1);
            int armPos = pivot.getCurrentPosition();
            double pid = controller.calculate(armPos, target1);
            double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

            double power = pid + ff;

            pivot.setPower(power);

            telemetry.addData("Pos", pivot.getCurrentPosition());
            telemetry.addData("Target", target1);


            telemetry.update();

        }
    }
}
