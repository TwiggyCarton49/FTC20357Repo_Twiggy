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



    public static double p1 = 0, i1 = 0, d1 = 0;

    public static double f1 = 0;

    public static int target1 = 425;
    public static double p2 = 0, i2 = 0, d2 = 0;

    public static double f2 = 0;

    public static int target2 = 250;

    private final double ticks_in_degrees = 1440/180;
    private DcMotorEx pivot, slideLeft,slideRight;
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p1,i1,d1);
        controller2 = new PIDController(p2,i2,d2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pivot = hardwareMap.get(DcMotorEx.class,"pivot");
        slideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        slideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");

        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()){
            controller.setPID(p1,i1,d1);
            int slidePos = slideLeft.getCurrentPosition();
            double pid = controller.calculate(slidePos, target1);
            double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

            double slidePower = pid + ff;

            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);

            controller2.setPID(p2,i2,d2);
            int armPos = slideLeft.getCurrentPosition();
            double pid2 = controller.calculate(armPos, target2);
            double ff2 = Math.cos(Math.toRadians(target2/ticks_in_degrees)) * f2;

            double pivotPower = pid2 + ff2;

            slideLeft.setPower(pivotPower);

            telemetry.addData("Slide Pos", slidePos);
            telemetry.addData("Slide Target", target1);
            telemetry.addData("pivot Pos", armPos);
            telemetry.addData("pivot Target", target2);
            telemetry.update();

        }
    }
}
