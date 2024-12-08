package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

public class TeleOpTest extends LinearOpMode {


    DcMotor frontLeftMotor; // roata fata dreapta
    DcMotor backLeftMotor; // roata spate dreapta
    DcMotor frontRightMotor; // roata fata dreapta
    DcMotor backRightMotor; // roata spate dreapta
    DcMotor motorIntake; // motorul care extinde glisiera de intake
    DcMotor motorOutake1; /// motor glisiera outtake 2
    DcMotor motorOutake2; // motor glisiera outtake 2
    Servo servoIntake1,servoIntake2; // lasa intake-ul sau urca intakeul (partea verde)
    DcMotor coreHexIntake; // motorul pentru periile de la intake
    Servo servoGrabber, // gheara cu care apuca elementul outtake-ul
            servoArmGrabber; // ridica gheara

    IMU imu;

    boolean active = false; // Starea glisierei (extins/retras)
    boolean activeIntakePull = false; // Stare intake pull (aprins/inchis)
    boolean activeIntakePush = false; // Stare intake push (aprins/inchis)
    boolean activeOutakeLiftForIntake = false;
    boolean activeGrabber = false; // start grabber (apuca/lasa)
    boolean activeArmGrabberUp = false;
    Timing.Timer delay = new Timing.Timer(200, TimeUnit.MILLISECONDS); // Timer pentru debounce
    Timing.Timer debounceTimer = new Timing.Timer(500, TimeUnit.MILLISECONDS); // Timer pentru debounceA
    Timing.Timer debounceTimerB = new Timing.Timer(500, TimeUnit.MILLISECONDS); // Timer pentru debounceB
    Timing.Timer debounceTimerX = new Timing.Timer(500, TimeUnit.MILLISECONDS); // Timer pentru debounceX
    Timing.Timer debounceTimerY = new Timing.Timer(500, TimeUnit.MILLISECONDS); // Timer pentru debounceY
    Timing.Timer debounceTimerRT = new Timing.Timer(500, TimeUnit.MILLISECONDS); // Timer pentru debounceRT
    Timing.Timer outakeTimer = new Timing.Timer(1000,TimeUnit.MILLISECONDS); //TImer pentru ridicare outake-ului inaite sa plece intake-ul
    final int forwardPosition = 1600; // Poziția extinsă
    final int backwardPosition = 0; // Poziția retrasă

    @Override
    public void runOpMode() throws InterruptedException {

        // Inițializarea hardware-ului pentru motoare
        frontLeftMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        backLeftMotor = hardwareMap.dcMotor.get("leftRearMotor");
        frontRightMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        backRightMotor = hardwareMap.dcMotor.get("rightRearMotor");
        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorOutake1 = hardwareMap.dcMotor.get("motorOutake1");
        motorOutake2 = hardwareMap.dcMotor.get("motorOutake2");
        coreHexIntake = hardwareMap.dcMotor.get("coreHexIntake");

        // Inițializarea hardware-ului pentru servo
        servoIntake1 = hardwareMap.servo.get("servoIntake1");
        servoIntake2 = hardwareMap.servo.get("servoIntake2");
        servoGrabber = hardwareMap.servo.get("servoGrabber"); // gheara cu care apuca elementul outtake-ul
        servoArmGrabber = hardwareMap.servo.get("servoArmGrabber"); // ridica gheara

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        servoIntake1.setDirection(Servo.Direction.REVERSE); // de la stanga la dreapta cum te uiti spre intake


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOutake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOutake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHexIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOutake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOutake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        coreHexIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        // Pornește timer-ul de debounce
        debounceTimer.start();
        debounceTimerB.start();
        debounceTimerX.start();
        debounceTimerY.start();
        debounceTimerRT.start();

        waitForStart(); // Așteaptă apăsarea butonului "Start" din Driver Station

        // Bucla principală
        while (opModeIsActive()) {
            double lt = gamepad1.left_trigger; // Actualizează constant valorile lui lt și rt
            double rt = gamepad1.right_trigger;

            intakeMove(lt, rt); // Control continuu pentru intake / BUTTON 'RT' AND 'LT'

            handleGlisiera();  // Control continuu pentru glisieră / BUTTON 'A'
            runIntake(); // Control continuu pentru periile de pe intake la tragerea elementelor din intake (pull) / BUTTON 'X'
            reverseIntake(); // Control continuu pentru periile de pe intake la impingerea elementelor din intake (push) / BUTTON 'B'
            outtakeGrabber(); // Daca apesi X prinde piesa iar la a 2 apasare il lasa jos / BUTTON 'Y'
            armGrabberMove(); // Daca apesi RT bratul de pe outtake se va ridica pentru a lasa piesa in cos / BUTTON 'RT'

            telemetry.addData("Glisiera Intake Active: ", active);
            telemetry.addData("Outake Lift For Intake: ",activeOutakeLiftForIntake);
            telemetry.addData("Intake Pull: ",activeIntakePull);
            telemetry.addData("Intake Push: ",activeIntakePush);
            telemetry.addData("Grabber Status: ", activeGrabber);
            telemetry.addData("Arm Grabber Up: ", activeArmGrabberUp);
            telemetry.update();
        }
    }

    //START SECTION BUTTON 'RT' AND 'LB'

    public void intakeMove(double lt, double rt) {  // daca apas rt intake-ul se va duce in fata, iar daca apas lt se va duce in spate
        int position = motorIntake.getCurrentPosition();

        if (position <= 1600 && (rt > 0 || lt > 0) && activeOutakeLiftForIntake) {
            motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorIntake.setPower(rt - lt);
        } else if (position >= 1600 && activeOutakeLiftForIntake) {
            motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorIntake.setPower(-1);
        }
    }

    //END SECTION BUTTON 'RT' AND 'LB'

    // START SECTION BUTTON 'A'

    public void handleGlisiera() {

        // Verifică dacă butonul `a` a fost apăsat și timer-ul de debounce a expirat
        if (gamepad1.a && debounceTimer.done()) {

            debounceTimer.start(); // Repornește timer-ul pentru următorul ciclu
            if (!active) { // Extinde glisiera
                liftOutakeForIntake();
                moveToPosition(forwardPosition);
                active = true;

            } else { // Retrage glisiera
                moveToPosition(backwardPosition);
                delay.start();

                while (!delay.done() && opModeIsActive()) {
                    telemetry.addData("Delay Status", "Waiting...");
                    telemetry.addData("Time Remaining", delay.remainingTime());
                    telemetry.update();
                }

                downOutakeForIntake();
                active = false;

            }
        }
    }

    public void moveToPosition(int targetPosition) {

        motorIntake.setTargetPosition(targetPosition);
        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorIntake.setPower(1);

        while (motorIntake.isBusy() && opModeIsActive()) {

            if (motorIntake.getCurrentPosition() >= 400){
                servoIntake1.setPosition(0.5);
                servoIntake2.setPosition(0.5);
            } else if (motorIntake.getCurrentPosition() < 400){
                servoIntake1.setPosition(0);
                servoIntake2.setPosition(0);
            }

            telemetry.addData("Motor Position", motorIntake.getCurrentPosition());
            telemetry.addData("Target Position", motorIntake.getTargetPosition());
            telemetry.update();
        }

        motorIntake.setPower(0); // Oprește motorul după atingerea poziției
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Target Position", "Done!");
        telemetry.update();
    }

    public void liftOutakeForIntake(){

        telemetry.addData("Status Outake: ", "Lifting outtake for intake!");
        telemetry.update();

        motorOutake1.setTargetPosition(800);
        motorOutake2.setTargetPosition(800);

        motorOutake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOutake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorOutake1.setPower(1);
        motorOutake2.setPower(1);

        while (motorOutake1.isBusy() && motorOutake2.isBusy() && opModeIsActive()) {
            telemetry.addData("Outtake Position", motorOutake1.getCurrentPosition());
            telemetry.update();
        }

        motorOutake1.setPower(0);
        motorOutake2.setPower(0);
        motorOutake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOutake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outakeTimer.start();
        while (!outakeTimer.done() && opModeIsActive()) {
            telemetry.addData("Waiting Timer", outakeTimer.remainingTime());
            telemetry.update();
        }

        activeOutakeLiftForIntake = true;

    }

    public void downOutakeForIntake(){

        telemetry.addData("Status Outake: ", "Lowering outtake for intake!");
        telemetry.update();

        motorOutake1.setTargetPosition(0);
        motorOutake2.setTargetPosition(0);

        motorOutake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOutake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorOutake1.setPower(1);
        motorOutake2.setPower(1);

        while (motorOutake1.isBusy() && motorOutake2.isBusy() && opModeIsActive()) {
            telemetry.addData("Outtake Position", motorOutake1.getCurrentPosition());
            telemetry.update();
        }

        motorOutake1.setPower(0);
        motorOutake2.setPower(0);
        motorOutake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOutake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        activeOutakeLiftForIntake = false;

    }

    // END SECTION BUTTON 'A'

    //START SECTION BUTTON 'X' // trage intake | pull = trage

    public void runIntake(){

        if (gamepad1.x && debounceTimerX.done()) {

            debounceTimerX.start();
            if (!activeIntakePull && !activeIntakePush) {

                coreHexIntake.setPower(1);
                activeIntakePull = true;

            } else if (activeIntakePull && !activeIntakePush) {

                coreHexIntake.setPower(0);
                activeIntakePull = false;
            }
        }

    }

    //END SECTION BUTTON 'X'

    //START SECTION BUTTON 'B' // scuipa intake (HAWK TUAH) | push = impinge = scuipa

    public void reverseIntake(){

        if (gamepad1.b && debounceTimerB.done()) {

            debounceTimerB.start();
            if (!activeIntakePush && !activeIntakePull) {

                coreHexIntake.setPower(-1);
                activeIntakePush = true;

            } else if(activeIntakePush && !activeIntakePull){

                coreHexIntake.setPower(0);
                activeIntakePush = false;
            }
        }

    }

    //END SECTION BUTTON 'B'

    // START SECTION BUTTON 'Y'

    public void outtakeGrabber(){

        if(gamepad1.y && debounceTimerY.done()){

            debounceTimerY.start();
            if (!activeGrabber) {

                servoGrabber.setPosition(0);
                activeGrabber = true;

            } else {

                servoGrabber.setPosition(0.5);
                activeGrabber = false;
            }

        }

    }

    // START SECTION BUTTON 'RB'

    public void armGrabberMove(){

        if(gamepad1.right_bumper && debounceTimerRT.done()) {

            debounceTimerRT.start();
            if (!activeArmGrabberUp) {

                servoArmGrabber.setPosition(0.5);
                activeArmGrabberUp = true;
            } else {
                servoArmGrabber.setPosition(0);
                activeArmGrabberUp = false;
            }
        }

    }

    //END SECTION BUTTON 'RB'

    // END SECTION BUTTON 'Y'

    public void stopAllMotor() {
        motorOutake1.setPower(0);
        motorOutake2.setPower(0);
        motorIntake.setPower(0);

        servoIntake1.setPosition(0);
        servoIntake2.setPosition(0);
        servoArmGrabber.setPosition(0);
        servoGrabber.setPosition(0);
    }

}
