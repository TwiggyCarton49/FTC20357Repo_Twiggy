package org.firstinspires.ftc.teamcode.Toros.Drive;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Toros.Util.BatteryClass;



@TeleOp(name = "MainDrive")
//Our Class for Drive Controlled period of the game
public class MainDrive extends LinearOpMode {

    /**
     * [PIDF controller] PIDF is a closed loop control which takes a proportional, integral, and derivative terms to calculate the error
     * as a difference of a target and will correct based on the terms.We use this to have precise control of our arm for our intake.
     **/
    private PIDController controller;
    private boolean Rtoggle,Xtoggle;
    //
    public static double p = 0.004, i = 0.001, d = 0.0005;
    public static double f = 0.195;
//    /**
//     P is for proportional which will be proportionate to the error this causes the arm to go up for us
//     I is for integral which integrates past values of error seeking to reduced the residual error by adding control and eliminate the error which gets us closer to the target point
//     D is for derivative which best estimates the trend of the error based on the rate of change to reduced the effect to dampen it to not overshoot
//     F is for feedforward which accounts for things more external and prevents disturbances in our use case showing gravity who is boss

    public static int target = 50;
    private final double ticks_in_degrees = 1440 / 180;// Ticks of the tetrix 60:1 motor divided by 180

    private DcMotor FrontLeftMotor, BackLeftMotor,FrontRightMotor,BackRightMotor,joint1;
    private VoltageSensor volt_prime;
    private Servo fingers,wrist,elbow;
    Gamepad currentGamepad = new Gamepad(),previousGamepad = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
//        controller = new PIDController(p, i, d); // <- Hey PID you should know what that is
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initializing Hardware in method down below called initHardware();
        initHardware();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                previousGamepad.copy(currentGamepad);
                currentGamepad.copy(gamepad1);

                drive();
                arm();
                claw();

                ///Battery power
                double volts = volt_prime.getVoltage();
                double battery = 0;
                if (volts > 12.00) {
                    battery = 100;
                } else if (volts <= 12) {
                    battery = (volts / 12.00) * 100;
                }

                telemetry.addData("Battery%", battery);
                telemetry.addData("motor power", joint1.getPower());
                initTelemetry();
                telemetry.update();

            }
        }
    }

        private void initHardware () {
            //Motors
            FrontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
            BackLeftMotor = hardwareMap.get(DcMotor.class, "bl");
            FrontRightMotor = hardwareMap.get(DcMotor.class, "fr");
            BackRightMotor = hardwareMap.get(DcMotor.class, "br");
            joint1 = hardwareMap.get(DcMotorEx.class, "joint1");
            fingers = hardwareMap.get(Servo.class,"fingers");
            wrist = hardwareMap.get(Servo.class,"wrist");
            elbow = hardwareMap.get(Servo.class,"elbow");


            FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            //Zero Power Behaviors
            FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //More servo stuff
            volt_prime = hardwareMap.get(VoltageSensor.class, "Control Hub");
        }


        private void initTelemetry () {

            BatteryClass battery = new BatteryClass(hardwareMap);
            telemetry.addData("Battery", battery.getBatteryPercent());
            telemetry.addData("Joint 1 pos", joint1.getCurrentPosition());

            telemetry.addData("Toggle",Xtoggle);
            telemetry.addData("Toggle",Rtoggle);
            telemetry.update();
        }
        private void drive () throws InterruptedException {


            if(currentGamepad.x && !previousGamepad.x){
                Xtoggle = !Xtoggle;
            }
            if(currentGamepad.b && !previousGamepad.b){
                Rtoggle = !Rtoggle;
            }

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            if(Xtoggle){
                x *= 0.75;
            }
            else{
                x*=1;
            }
            if(Rtoggle){
                turn *= 0.75;
            }
            else{
                turn*=1;
            }


            //Drive variables used in the calculations to run our motors
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);
            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            /**
             In basics this is taking the x and y of the left stick making them into an angle
             with the power being the hypot which is the square root of the sum of squares of the inputs
             more info here https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/hypot
             then takes the sin and cos of the angle making sure to convert to radians. It then creates a max
             using the absolute value of the sin and cos.

             The idea is that where you are going is angle theta with each wheel being a vector and when combined make the target direction when rotated 45 degrees

             Found on YT www.youtube.com/watch?v=gnSW2QpkGXQ which is a video about coding for mecanum drive wheels
             */


            //Calculations for our drive motors

            double fl = (power * cos / max + turn);
            double fr = (power * sin / max - turn);
            double bl = (power * sin / max + turn);
            double br = (power * cos / max - turn);

            /**
             In continuation the power is then calculated with the angles multiplied by the sin or cos divided the difference or sum of the max and turn
             */

            //If statement below is to make sure one motor does not exceed the power limit making it scale down

            if ((power + Math.abs(turn)) > 1) {
                fl /= power + Math.abs(turn);
                fr /= power + Math.abs(turn);
                bl /= power + Math.abs(turn);
                br /= power + Math.abs(turn);
            }



            //Motor Drive
            FrontLeftMotor.setPower(fl);
            FrontRightMotor.setPower(fr);
            BackLeftMotor.setPower(bl);
            BackRightMotor.setPower(br);
        }
        private void arm() {
            controller = new PIDController(p,i,d);
            double arm2 = gamepad2.left_stick_y;

                double powerA = 0;
                int armPos = joint1.getCurrentPosition();
                if(joint1.getCurrentPosition() < -950){
                    target = -950;
                }



                controller.setPID(p, i, d); // sets the terms
                double pid = controller.calculate(armPos, target); /// Remember that very funny equation for PID. Well I told the computer to do my math homework
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
                powerA = pid + ff;


               if(gamepad2.right_stick_y <= 1.0 && gamepad2.right_stick_y != 0.0|| gamepad2.right_stick_y >= -1.0 && gamepad2.right_stick_y != 0){
                    powerA = gamepad2.right_stick_y * 0.5;
                    target = armPos;
                }

               joint1.setPower(powerA);

        }
        private void claw(){
            fingers.setPosition(gamepad2.right_trigger);
        }
    }

//:3