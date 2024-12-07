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



    public static double p1 = 0.009, i1 = 0.001, d1 = 0.0005;

    public static double f1 = 0.185;

    public static int target1 = 425;
    public static double p2 = 0.04, i2 = 0.0, d2 = 0.0001;

    public static double f2 = 0.003;

    public static int target2 = 250;

    private final double ticks_in_degrees = 1440/180;
    private DcMotorEx ArmPivot1;
    private DcMotorEx Joint2;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p1,i1,d1);
        controller2 = new PIDController(p2,i2,d2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ArmPivot1 = hardwareMap.get(DcMotorEx.class,"joint1");
        Joint2 = hardwareMap.get(DcMotorEx.class,"joint2");

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

            controller2.setPID(p2,i2,d2);
            int armPos2 = Joint2.getCurrentPosition();
            double pid2 = controller2.calculate(armPos2, target2);
            double ff2 = Math.cos(Math.toRadians(target2/ticks_in_degrees)) * f2;

            double power2 = pid2 + ff2;

            Joint2.setPower(power2);

            telemetry.addData("Pos2", armPos2);
            telemetry.addData("Target2", target1);

//


            telemetry.update();

        }
    }
}
