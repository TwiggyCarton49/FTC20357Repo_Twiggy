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

/**
 * FEATURE TO DO LIST
 * (For the future of what the coding team needs to add)
 *
 * Hard Limit forward and back
 *
 *
 */

@TeleOp(name = "MainDrive")
//Our Class for Drive Controlled period of the game
public class MainDrive extends LinearOpMode {

    /**
     * [PIDF controller] PIDF is a closed loop control which takes a proportional, integral, and derivative terms to calculate the error
     * as a difference of a target and will correct based on the terms.We use this to have precise control of our arm for our intake.
     **/
    private PIDController controller;
    private PIDController controller2;
    //
    public static double p = 0.004, i = 0.001, d = 0.0005;
    public static double f = 0.195;
    public static double p2 = 0.004, i2 = 0.001, d2 = 0.0005;
    public static double f2 = 0.185;
    //
//    /**
//     P is for proportional which will be proportionate to the error this causes the arm to go up for us
//     I is for integral which integrates past values of error seeking to reduced the residual error by adding control and eliminate the error which gets us closer to the target point
//     D is for derivative which best estimates the trend of the error based on the rate of change to reduced the effect to dampen it to not overshoot
//     F is for feedforward which accounts for things more external and prevents disturbances in our use case showing gravity who is boss
//
//     PID equation  https://images.squarespace-cdn.com/content/v1/5230e9f8e4b06ab69d1d8068/1598232682278-PAUNGGGYUP19WS8C7TWN/PID+equation.png?format=1000w
//     Can't write it here because it is too big and has an integral
//     */
    public static int target = 50;
    public static int target2 = 450;
    private final double ticks_in_degrees = 1440 / 180;
    //This the ticks that the Tetrix motor does in degrees (dividing by 180) which is part of the PIDF Calculation

    //Declares the Variables for all of our motors and servos
    private DcMotor extensionMotor;
    private DcMotor FrontLeftMotor;
    private VoltageSensor volt_prime;
    private DcMotor joint1, joint2;
    private DcMotor BackLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackRightMotor;

    boolean Xtoggle = false;
    boolean Rtoggle = false;
    private Servo Claw1;
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();
    //Variable above is used for controlling the speed of our drivetrain

    //This below runs the OpMode/Program for our robot
    @Override
    public void runOpMode() throws InterruptedException {
        //Creation for instance of PIDF controller and Telemetry using FTC Dash
//        controller = new PIDController(p, i, d); // <- Hey PID you should know what that is
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initializing Hardware in method down below called initHardware();
        initHardware();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                previousGamepad.copy(currentGamepad);
                currentGamepad.copy(gamepad1);
                //Gamepad controls
                /**
                 * GAMEPAD 1 CONTROLS
                 * The controls for the robot
                 * Left stick X (left to right is to move horizontally on the field)
                 * Left stick Y (up and down is to move vertically on the field)
                 * Right Stick X (rotates the robot left and right)
                 * DPAD Up to increase speed if not already at 100%
                 * DPAD Down to decrease speed if not already at 0%
                 * DPAD Left to set to 1% (Can't be zero or else we won't be able to move)
                 * DPAD Right to set to 100%
                 *
                 * GAMEPAD 2 CONTROLS
                 * Left Bumper to open claw
                 * Right Bumper to close claw
                 * X Y A B to go to a certain position on the arm
                 * (Other controls work in progress as the season goes on)
                 */



                drive();
                arm();
                claw();
                //Speed tuning if you want to be slow
                //arm();

                //Servo Control


//                Arm1.setPower(gamepad2.left_stick_y);


//
//                if (target > -2100){
//                   f =  0.05;
//                } else if (target < -2100) {
//                    f = -0.05;
//                }
//                /*Allows for our robot to hold the position of the arm when passing a certain point by
//                multiplying the f value by -1 which allows for the arm to be perfectly stable no matter if it is behind or in front of the robot
//                */
//
//
//                //Useful for later
//                double powerA = 0;
//                int armPos = ArmPivot1.getCurrentPosition();
//
//                //Now the fun begins
//                controller.setPID(p, i, d); // sets the terms
//                double pid = controller.calculate(armPos, target); /// Remember that very funny equation for PID. Well I told the computer to do my math homework
//                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f; // Creates a number to get an angle related to the target and ticks and muliplies by our f term
//                powerA = pid + ff; // Gives the power to the motor
////
////                // This below is a fun little statement that when the stick is not equal to 0 and is either below 1 or above -1 will give power to the arm divided by 2
////
//                if(gamepad2.left_stick_y <= 1.0 && gamepad2.left_stick_y != 0.0|| gamepad2.left_stick_y >= -1.0 && gamepad2.left_stick_y != 0){
//                    powerA = gamepad2.left_stick_y / 0.5;
//                    target = armPos;
//                }
////
////                // Arm power
//                ArmPivot1.setPower(powerA);
//
//                //ArmControl
//                if (gamepad2.y) {
//                    target = -2850;
//                } else if (gamepad2.x) {
//                    target = -1900;
//                } else if (gamepad2.b){
//                    target = -550;
//                } else if(gamepad2.a){
//                    target =-3250;
//                }

                //basic arm control


                ///Battery power
                double volts = volt_prime.getVoltage();
                double battery = 0;
                if (volts > 12.00) {
                    battery = 100;
                } else if (volts <= 12) {
                    battery = (volts / 12.00) * 100;
                }

                telemetry.addData("Battery%", battery);
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
            joint2 = hardwareMap.get(DcMotorEx.class, "joint2");
            extensionMotor = hardwareMap.get(DcMotor.class,"extend");

            Claw1 = hardwareMap.get(Servo.class, "claw");
            Claw1.setPosition(1);


            FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            //Zero Power Behaviors
            FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //More servo stuff

            Claw1.setPosition(1);


            volt_prime = hardwareMap.get(VoltageSensor.class, "Control Hub");
        }


        private void initTelemetry () {


//        telemetry.addData("Target", target);
            BatteryClass battery = new BatteryClass(hardwareMap);
            telemetry.addData("Battery", battery.getBatteryPercent());
            telemetry.addData("Joint 1 pos", joint1.getCurrentPosition());
            telemetry.addData("Joint 2 pos", joint2.getCurrentPosition());
            telemetry.addData("extension pos", extensionMotor.getCurrentPosition());
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
            int SlidePos = extensionMotor.getCurrentPosition();
//            double elevatorControl = gamepad2.left_stick_y;
            double arm2 = gamepad2.left_stick_y;
//            if (target > -2100){
//                  f =  0.05;
//                } else if (target < -2100) {
//                    f = -0.05;
//                }
                /*Allows for our robot to hold the position of the arm when passing a certain point by
                multiplying the f value by -1 which allows for the arm to be perfectly stable no matter if it is behind or in front of the robot
                */

             //Useful for later
                double powerA = 0;
                int armPos = joint1.getCurrentPosition();
                if(joint1.getCurrentPosition() < -950){
                    target = -950;
                }


//                //Now the fun begins
                controller.setPID(p, i, d); // sets the terms
                double pid = controller.calculate(armPos, target); /// Remember that very funny equation for PID. Well I told the computer to do my math homework
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f; // Creates a number to get an angle related to the target and ticks and muliplies by our f term
                powerA = pid + ff; // Gives the power to the motor
                // This below is a fun little statement that when the stick is not equal to 0 and is either below 1 or above -1 will give power to the arm divided by 2

               if(gamepad2.right_stick_y <= 1.0 && gamepad2.right_stick_y != 0.0|| gamepad2.right_stick_y >= -1.0 && gamepad2.right_stick_y != 0){
                    powerA = gamepad2.right_stick_y * 0.5;
                    target = armPos;
                }

                // Arm power
               joint1.setPower(powerA);

            controller2 = new PIDController(p2,i2,d2);
             int target2 = joint2.getCurrentPosition();
//            if (target > -2100){
//                  f =  0.05;
//                } else if (target < -2100) {
//                    f = -0.05;
//                }
                /*Allows for our robot to hold the position of the arm when passing a certain point by
                multiplying the f value by -1 which allows for the arm to be perfectly stable no matter if it is behind or in front of the robot
                */

            //Useful for later
            double powerA2 = 0;
            int armPos2 = joint2.getCurrentPosition();
            

//                //Now the fun begins
            controller2.setPID(p2, i2, d2); // sets the terms
            double pid2 = controller2.calculate(armPos2, target2); /// Remember that very funny equation for PID. Well I told the computer to do my math homework
            double ff2 = Math.cos(Math.toRadians(target2 / ticks_in_degrees)) * f2; // Creates a number to get an angle related to the target and ticks and muliplies by our f term
            powerA2 = pid2 + ff2; // Gives the power to the motor
            // This below is a fun little statement that when the stick is not equal to 0 and is either below 1 or above -1 will give power to the arm divided by 2

            if(gamepad2.left_stick_y <= 1.0 && gamepad2.left_stick_y != 0.0|| gamepad2.left_stick_y >= -1.0 && gamepad2.left_stick_y != 0){
                powerA2 = gamepad2.left_stick_y * 0.5;
                target2 = armPos2;
            }

            // Arm power
            joint2.setPower(powerA2);

//            joint1.setPower(elevatorControl / 2);


            if(gamepad2.a){
                if(SlidePos >= 2650)
                extensionMotor.setPower(0);
                else{
                    extensionMotor.setPower(-0.5);

                }
            }   
            if(gamepad2.b){
                if(SlidePos <= 2090)
                    extensionMotor.setPower(0);
                else{
                    extensionMotor.setPower(0.5);

                }
            }

        }
        private void claw(){
            if (gamepad2.left_bumper) {
                Claw1.setPosition(0);
            } else if (gamepad2.right_bumper) {
                Claw1.setPosition(1);
            }
        }
    }

//:3