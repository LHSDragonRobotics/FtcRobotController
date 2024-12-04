package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "MainDrive")
public class MainDrive extends LinearOpMode {

    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        double offset = 0;

        imu.initialize(new BNO055IMU.Parameters());

        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Read the gamepad inputs
                Orientation pose = imu.getAngularOrientation();
                double yaw = pose.firstAngle * 60; // Calculate yaw from angles provided by the IMU
                double yawRadians = Math.toRadians(yaw - offset);
                double x = -(Math.sin(yawRadians) * gamepad1.left_stick_x + Math.cos(yawRadians) * gamepad1.left_stick_y);
                double y = Math.cos(yawRadians) * -gamepad1.left_stick_x + Math.sin(yawRadians) * gamepad1.left_stick_y;
                double rotation = -gamepad1.right_stick_x;

                if (gamepad1.left_bumper)
                    offset = yaw;

                // Drive the robot
                frontLeft.setPower(x - y - rotation);
                frontRight.setPower(x + y + rotation);
                backLeft.setPower(x + y - rotation);
                backRight.setPower(x - y + rotation);
                telemetry.addData("X", x);
                telemetry.addData("Y", y);
                telemetry.addData("rot", rotation);
                telemetry.addData("Left Pow", backLeft.getPower());
                telemetry.addData("Right Pow", backRight.getPower());
                telemetry.addData("Front Left Pow", frontLeft.getPower());
                telemetry.addData("Front Right Pow", frontRight.getPower());
                telemetry.addData("Yaw", yaw - offset);
                telemetry.addData("Left X", gamepad1.left_stick_x);
                telemetry.addData("Left Y", gamepad1.left_stick_y);
                telemetry.addData("Skibidi toilet", "1");

                telemetry.update();
            }
        }
    }
}
