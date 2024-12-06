package org.firstinspires.ftc.teamcode.Toros.Autonomous;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
@Autonomous(name = "AutoV1")
//Autonomous for first meet of 2024/25
public class AutoV1 extends LinearOpMode{
    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackRightMotor;

    private PIDController controller;
    private PIDController controller2;



    public static double p1 = -0.04, i1 = 0.001, d1 = 0.0005;

    public static double f1 = 0.185;

    public static int target1 = 250;
    public static double p2 = 0.0, i2 = 0.0, d2 = 0.0;

    public static double f2 = 0;

    public static int target2 = 250;

    private final double ticks_in_degrees = 1440/180;
    private DcMotorEx Joint1;
    private DcMotorEx Joint2;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        //power motors
        runMotors(1, 1, 1, 1, 1200);
        arm(-650, 200);

    }
    private void initHardware(){
        //Motors
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        BackRightMotor = hardwareMap.get(DcMotor.class, "br");
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller = new PIDController(p1,i1,d1);
        Joint1 = hardwareMap.get(DcMotorEx.class,"joint1");
        Joint2 = hardwareMap.get(DcMotorEx.class,"joint2");

    }
    private void runMotors(double p1, double p2, double p3, double p4, int sleepTime){
        FrontLeftMotor.setPower(p1);
        FrontRightMotor.setPower(p2);
        BackLeftMotor.setPower(p3);
        BackRightMotor.setPower(p4);
        sleep(sleepTime);
    }
    private void arm(int targetPos, int targetPos2){
        controller.setPID(p1,i1,d1);
        int armPos = Joint1.getCurrentPosition();
        double pid = controller.calculate(armPos, targetPos);
        double ff = Math.cos(Math.toRadians(targetPos/ticks_in_degrees)) * f1;

        double power = pid + ff;
        Joint1.setPower(power);

              controller2.setPID(p2,i2,d2);
            int armPos2 = Joint2.getCurrentPosition();
            double pid2 = controller2.calculate(armPos2, targetPos2);
            double ff2 = Math.cos(Math.toRadians(targetPos2/ticks_in_degrees)) * f2;

            double power2 = pid2 + ff2;

            Joint2.setPower(power2);

    }
}
