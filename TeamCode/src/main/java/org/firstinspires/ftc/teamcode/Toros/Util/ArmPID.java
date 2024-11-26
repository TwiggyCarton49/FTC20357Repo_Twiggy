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



    public static double p1 = 0, i1 = 0, d1 = 0;

    public static double f1 = 0;

    public static int target1 = 5;

    private final double ticks_in_degrees = 1440/180;
    private DcMotorEx ArmPivot1;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p1,i1,d1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ArmPivot1 = hardwareMap.get(DcMotorEx.class,"pivot1");


        waitForStart();
        while (opModeIsActive()){
            controller.setPID(p1,i1,d1);
            int armPos = ArmPivot1.getCurrentPosition();
            double pid = controller.calculate(armPos, target1);
            double ff = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

            double power = pid + ff;

            ArmPivot1.setPower(power);

            telemetry.addData("Pos", armPos);
            telemetry.addData("Target", target1);

            telemetry.update();

        }
    }
}
