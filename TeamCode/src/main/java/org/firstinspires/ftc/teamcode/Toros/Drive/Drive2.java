package org.firstinspires.ftc.teamcode.Toros.Drive;

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
public class Drive2 extends LinearOpMode {

    /**[PIDF controller] PIDF is a closed loop control which takes a proportional, integral, and derivative terms to calculate the error
     as a difference of a target and will correct based on the terms.We use this to have precise control of our arm for our intake.
    **/
//    private PIDController controller;
//
//    public static double p = 0.03, i = 0.0022, d = 0.001;
//    public static double f = -0.05;
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
//    public static int target = -100;
//   private final double ticks_in_degrees = 1440 / 180;
    //This the ticks that the Tetrix motor does in degrees (dividing by 180) which is part of the PIDF Calculation

    //Declares the Variables for all of our motors and servos
    private VoltageSensor volt_prime;
    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackRightMotor;
    private DcMotor ArmPivot1;
    private DcMotor Elevator;
    private DcMotor Elevator2;
    private Servo Claw1;


    double speed = 100;
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

                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;

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
                FrontLeftMotor.setPower(fl * (speed/100));
                FrontRightMotor.setPower(fr * (speed/100));
                BackLeftMotor.setPower(bl * (speed/100));
                BackRightMotor.setPower(br * (speed/100));
                //Speed tuning if you want to be slow


                //Servo Control
                Claw1.setPosition((gamepad2.right_stick_y * 180));

//                Arm1.setPower(gamepad2.left_stick_y);

                } if (gamepad1.dpad_down) {
                    speed -= 2;
                } else if (gamepad1.dpad_up) {
                    speed += 2;
                } else if (gamepad1.dpad_left) {
                    speed = 1;
                } else if (gamepad1.dpad_right){
                    speed = 100;
                }if (speed > 100){
                    speed = 100;
                } else if (speed < 2){
                    speed = 2;
                }


//
//                if (target > -2100){
//                    f =  0.05;
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
//                int armPos = Arm1.getCurrentPosition();
//
//                //Now the fun begins
//                controller.setPID(p, i, d); // sets the terms
//                double pid = controller.calculate(armPos, target); /// Remember that very funny equation for PID. Well I told the computer to do my math homework
//                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f; // Creates a number to get an angle related to the target and ticks and muliplies by our f term
//                powerA = pid + ff; // Gives the power to the motor
//
//                // This below is a fun little statement that when the stick is not equal to 0 and is either below 1 or above -1 will give power to the arm divided by 2
//
//                if(gamepad2.left_stick_y <= 1.0 && gamepad2.left_stick_y != 0.0|| gamepad2.left_stick_y >= -1.0 && gamepad2.left_stick_y != 0){
//                    powerA = gamepad2.left_stick_y / 0.5;
//                    target = armPos;
//                }
//
//                // Arm power
//                Arm1.setPower(powerA);
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
            double elevator = gamepad2.left_stick_y;
            double pivot = gamepad2.left_stick_x;

            Elevator.setPower(elevator);
            ArmPivot1.setPower(pivot);

                ///Battery power
                double volts = volt_prime.getVoltage();
                double battery = 0;
                if(volts > 12.00){
                    battery = 100;
                } else if (volts <= 12) {
                    battery = (volts / 12.00) * 100;
                }
                
                telemetry.addData("Battery%",battery);
                telemetry.addData("Speed", speed);
                initTelemetry();
                telemetry.update();

            }
        }
    //}

    private void initHardware(){
        //Motors
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "m1");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "m2");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "m3");
        BackRightMotor = hardwareMap.get(DcMotor.class, "m4");


        Claw1 = hardwareMap.get(Servo.class, "Claw1");

        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Zero Power Behaviors
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //More servo stuff

        Claw1.setPosition(0);
        ArmPivot1 = hardwareMap.get(DcMotor.class,"Pivot1");
        Elevator = hardwareMap.get(DcMotor.class, "elevator");

        ArmPivot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        volt_prime = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }


    private void initTelemetry() {
        telemetry.addData("claw angle", Claw1.getPosition() * 180);


//        telemetry.addData("Target", target);
//        telemetry.addData("ArmPosition", Arm1.getCurrentPosition());
//        telemetry.addData("Pos", Arm1.getCurrentPosition());
        telemetry.addData("Speed", speed);
        telemetry.update();
    }
}