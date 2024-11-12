package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp
public class OdometryUnitTest extends LinearOpMode {

    Motor lf,lr,rf,rr;
    MecanumDrive drive;
    GamepadEx driverOp;

    HolonomicOdometry robotOdometry;
    OdometrySubsystem odometry;
    PurePursuitCommand ppCmd;

    public double TRACKWIDTH = 1;
    public double TICKS_TO_CM = 372.52;
    public double WHEEL_DIAMETER = 3.5;
    public double CENTER_WHEEL_OFFSET = 0;

    MotorEx verticalEncoder, horizontalEncoder;

    IMU imu;

    public void initHW(){
        lf = new Motor(hardwareMap, "leftFront");
        lr = new Motor(hardwareMap, "leftRear");
        rf = new Motor(hardwareMap, "rightFront");
        rr = new Motor(hardwareMap, "rightRear");

        drive = new MecanumDrive(lf, rf, lr, rr);
        driverOp = new GamepadEx(gamepad1);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        robotOdometry = new HolonomicOdometry(
                () -> (verticalEncoder.getCurrentPosition() +
                        imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle) * TICKS_TO_CM,
                () -> (verticalEncoder.getCurrentPosition() -
                        imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle) * TICKS_TO_CM,
                () -> horizontalEncoder.getCurrentPosition() * TICKS_TO_CM,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        odometry = new OdometrySubsystem(robotOdometry);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        try{

            initHW();

            waitForStart();

            ppCmd = new PurePursuitCommand(
                    drive, odometry,
                    new StartWaypoint(odometry.getPose()),
                    new GeneralWaypoint(
                            90,-46, Math.toRadians(75),
                            0.6, 0.5, 30)

            );

            throw new InterruptedException();
        }
        catch (InterruptedException e){

        }

    }
}
