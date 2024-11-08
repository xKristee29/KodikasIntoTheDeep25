package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp
public class TeleOpMecanum extends LinearOpMode {

    Motor lf,lr,rf,rr;
    MecanumDrive drive;
    GamepadEx driverOp;
    DcMotor motorGlisiera;
    Servo servoGlisiera;
    CRServo servoIntake;


    boolean intakeExtended = false;

    Timing.Timer intakeTimer = new Timing.Timer(1, TimeUnit.MILLISECONDS);
    Timing.Timer intakeCooldown = new Timing.Timer(1, TimeUnit.MILLISECONDS);

    int posEnb = 1600;
    int threshold = 100;

    public void intakeEnb(){

        if(gamepad2.a && !intakeExtended && intakeCooldown.done()){

            intakeExtended = true;
            motorGlisiera.setTargetPosition(posEnb);
            motorGlisiera.setPower(1);
            motorGlisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            intakeCooldown = new Timing.Timer(1, TimeUnit.MILLISECONDS);
            intakeTimer = new Timing.Timer(600, TimeUnit.MILLISECONDS);

        } else if (gamepad2.a && intakeExtended && intakeCooldown.done()) {

            intakeExtended = false;
            motorGlisiera.setTargetPosition(0);
            motorGlisiera.setPower(1);
            motorGlisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            intakeCooldown = new Timing.Timer(1, TimeUnit.MILLISECONDS);
            intakeTimer = new Timing.Timer(600, TimeUnit.MILLISECONDS);
        }

        if(intakeExtended &&
                Math.abs(motorGlisiera.getCurrentPosition() - posEnb) <= threshold){
            servoGlisiera.setPosition(1);
        }

        if(!intakeExtended){
            servoGlisiera.setPosition(0.5);
        }

    }

    public void intakeMove(double lt, double rt){
        if (motorGlisiera.getCurrentPosition() - posEnb > -100 && intakeTimer.done()) {
            motorGlisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorGlisiera.setPower(rt-lt);
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {

        try{

            lf = new Motor(hardwareMap, "leftFront");
            lr = new Motor(hardwareMap, "leftRear");
            rf = new Motor(hardwareMap, "rightFront");
            rr = new Motor(hardwareMap, "rightRear");

            motorGlisiera.setPower(0);
            motorGlisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorGlisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            drive = new MecanumDrive(lf, rf, lr, rr);
            driverOp = new GamepadEx(gamepad1);

            waitForStart();

            double speed = -1;

            while(!isStopRequested()){
                drive.driveRobotCentric(
                    driverOp.getLeftX()*speed,
                    driverOp.getLeftY()*speed,
                    driverOp.getRightX()*speed
                );

                intakeEnb();

               intakeMove(gamepad2.left_trigger,gamepad2.right_trigger);


            }

            throw new InterruptedException();
        }
        catch (InterruptedException e){

        }
    }
}
