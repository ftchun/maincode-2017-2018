// Importing the first package
package org.firstinspires.ftc.teamcode;

// Importing the information for stuff
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "Moar auton red 1", group = "Linear")
public class MoarAutonRed1 extends LinearOpMode {

    private DcMotor motorFL; //front left
    private DcMotor motorFR; //front right
    private DcMotor motorBL; //back left
    private DcMotor motorBR; //back right

    private DcMotor motorArm; //arm swinging motor

    private Servo servoL;
    private Servo servoR;

    @Override
    public void runOpMode() throws InterruptedException {

        //assign motor vars to hardware configuration
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorArm = hardwareMap.dcMotor.get("motorArm");

        servoL = hardwareMap.servo.get("servoL");
        servoR = hardwareMap.servo.get("servoR");

        //reverse motors for left wheels
        motorArm.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);


        //set modes for motors
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        waitForStart();


        //MAIN CODE
        gotoArmPosition(200, .5);
        timer(1000);

        gotoServoPosition(0.7);
        timer(1000);

        gotoArmPosition(500, .5);
        timer(1000);

        driveForward(2200, .05);
        timer(3000);
        
        turnLeft(900, .25);
        timer(1000);
        
        driveForward(300, .25);
        timer(1000);
        
        gotoServoPosition(.1);
        timer(500);
        
        gotoServoPosition(0.7);
        timer(1000);
        
        driveBackward(-300, -.05);
        timer(3000);
        
        gotoServoPosition(.1);
        timer(500);
        
        driveBackward(100, .05);
        timer(3000);
        
        gotoArmPosition(150, .5);
        timer(3000);
        
        driveForward(300, .25);
        timer(1000);
        
    }
    private void driveForward (int position, double power) throws InterruptedException {
        while (motorBR.getCurrentPosition() < position || motorBL.getCurrentPosition() < position) {
            motorFL.setPower(power);
            motorFR.setPower(power);
            motorBL.setPower(power);
            motorBR.setPower(power);
        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void driveBackward(int position, double power) {
        while (motorBR.getCurrentPosition() > position && motorBL.getCurrentPosition() > position) {
            motorFL.setPower(power);
            motorFR.setPower(power);
            motorBL.setPower(power);
            motorBR.setPower(power);
        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void turnLeft (int position, double power) {
        while (motorBR.getCurrentPosition() < position) {
            motorFR.setPower(power);
            motorBR.setPower(power);
        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    private void turnRight (int position, double power) {
        while (motorBL.getCurrentPosition() < position) {
            motorFL.setPower(power);
            motorBL.setPower(power);
        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void gotoServoPosition (double position) {
        //value from 0 to 0.8 (corresponds to servoL position)
        servoL.setPosition(position);
        servoR.setPosition(0.9-position);
    }

    private void gotoArmPosition (int position, double power) {
        motorArm.setTargetPosition(position);
        motorArm.setPower(power);
    }

    private void timer (long time) throws InterruptedException {
        Thread.sleep(time);
    }
}
