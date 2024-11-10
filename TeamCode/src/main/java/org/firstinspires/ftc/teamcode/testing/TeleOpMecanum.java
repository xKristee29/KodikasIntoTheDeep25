package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

    DcMotor glisiera1, glisiera2;

    boolean intakeExtended = false;

    Timing.Timer intakeTimer = new Timing.Timer(1, TimeUnit.MILLISECONDS);
    Timing.Timer intakeCooldown = new Timing.Timer(1, TimeUnit.MILLISECONDS);

    Timing.Timer intakeIn = new Timing.Timer(1, TimeUnit.MILLISECONDS);
    Timing.Timer intakeOut = new Timing.Timer(1, TimeUnit.MILLISECONDS);

    int posEnb = 280;
    int threshold = 20;

    public void intakeEnb(){

        if(gamepad1.a && !intakeExtended && intakeCooldown.done()){

            intakeExtended = true;
            motorGlisiera.setTargetPosition(posEnb);
            motorGlisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorGlisiera.setPower(1);
            servoIntake.setPower(1);

            intakeTimer = new Timing.Timer(600, TimeUnit.MILLISECONDS);
            intakeTimer.start();

            intakeCooldown = new Timing.Timer(200, TimeUnit.MILLISECONDS);
            intakeCooldown.start();

        } else if (gamepad1.a && intakeExtended && intakeCooldown.done()) {

            intakeExtended = false;
            motorGlisiera.setTargetPosition(0);
            motorGlisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorGlisiera.setPower(1);

            servoIntake.setPower(0);

            intakeTimer = new Timing.Timer(600, TimeUnit.MILLISECONDS);
            intakeTimer.start();

            intakeCooldown = new Timing.Timer(200, TimeUnit.MILLISECONDS);
            intakeCooldown.start();
        }

        if(intakeExtended &&
                Math.abs(motorGlisiera.getCurrentPosition() - posEnb) <= threshold){
            servoGlisiera.setPosition(0.4);
        }

        if(!intakeExtended){
            servoGlisiera.setPosition(0);
        }

    }

    public void intakeMove(double lt, double rt){
        if (rt - lt != 0 && intakeTimer.done()) {
            motorGlisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorGlisiera.setPower(rt-lt);
        }

    }

    public void intakeToggle(){
        if(gamepad1.x && intakeIn.done()){
            servoIntake.setPower(1-servoIntake.getPower());

            intakeIn = new Timing.Timer(200, TimeUnit.MILLISECONDS);
            intakeIn.start();
        }
        if(gamepad1.b && intakeOut.done()){
            servoIntake.setPower(-(1-Math.abs(servoIntake.getPower())));

            intakeOut = new Timing.Timer(200, TimeUnit.MILLISECONDS);
            intakeOut.start();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        try{

            lf = new Motor(hardwareMap, "leftFront");
            lr = new Motor(hardwareMap, "leftRear");
            rf = new Motor(hardwareMap, "rightFront");
            rr = new Motor(hardwareMap, "rightRear");

            servoGlisiera = hardwareMap.servo.get("servoGlisiera");
            servoIntake = hardwareMap.crservo.get("servoIntake");

            glisiera1 = hardwareMap.dcMotor.get("glisiera1");
            glisiera2 = hardwareMap.dcMotor.get("glisiera2");

            motorGlisiera = hardwareMap.dcMotor.get("motorGlisiera");
            motorGlisiera.setPower(0);
            motorGlisiera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorGlisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorGlisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            drive = new MecanumDrive(lf, rf, lr, rr);
            driverOp = new GamepadEx(gamepad1);

            intakeTimer = new Timing.Timer(1, TimeUnit.MILLISECONDS);
            intakeTimer.start();

            intakeCooldown = new Timing.Timer(1, TimeUnit.MILLISECONDS);
            intakeCooldown.start();

            intakeIn = new Timing.Timer(1, TimeUnit.MILLISECONDS);
            intakeIn.start();

            intakeOut = new Timing.Timer(1, TimeUnit.MILLISECONDS);
            intakeOut.start();

            waitForStart();

            double speed = -1;

            while(!isStopRequested()){
                drive.driveRobotCentric(
                    driverOp.getLeftX()*speed,
                    driverOp.getLeftY()*speed,
                    driverOp.getRightX()*speed
                );

                intakeEnb();

                intakeMove(gamepad1.left_trigger,gamepad1.right_trigger);

                intakeToggle();

                //glisiera1.setPower(-gamepad1.left_stick_y);
                if(gamepad1.dpad_up) glisiera1.setPower(-0.5);
                else if(gamepad1.dpad_down) glisiera1.setPower(0.5);
                else glisiera1.setPower(0);

                telemetry.addData("Pos",motorGlisiera.getCurrentPosition());
                telemetry.addData("Ext",intakeExtended);
                telemetry.addData("Cooldown",intakeCooldown.done());
                telemetry.addData("Timer",intakeTimer.done());
                telemetry.addData("In",intakeIn.done());
                telemetry.addData("Out",intakeOut.done());
                telemetry.addData("G1",glisiera1.getCurrentPosition());
                telemetry.addData("G2",glisiera2.getCurrentPosition());
                telemetry.update();
            }

            throw new InterruptedException();
        }
        catch (InterruptedException e){

        }
    }
}
