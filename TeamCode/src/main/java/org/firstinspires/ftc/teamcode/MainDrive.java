package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "MainDrive")
public class MainDrive extends LinearOpMode {

    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor armRotate;
    private DcMotor armSlide;
    private Servo handServo;
    private Servo handWristServo;
    private Servo tiltServo;
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armSlide = hardwareMap.get(DcMotor.class, "armSlide");
        handServo = hardwareMap.get(Servo.class, "handServo");
        handWristServo = hardwareMap.get(Servo.class, "wristServo");
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        TelemetryMode telemetryMode = TelemetryMode.ALL;
        double offset = 0;
        double armRotationOffset = armRotate.getCurrentPosition();
        double armTarget = 0;
        double armSlideOffset = armSlide.getCurrentPosition();
        double armSlideTarget = 0;
        double handServoAngle = 1;
        double handWristServoAngle = 0;
        double tiltServoAngle = 0;
        boolean updatePressed = false;

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
            while (opModeIsActive()) {
                double armAngle = (armRotate.getCurrentPosition() - armRotationOffset);
                double armSlideLength = (armSlide.getCurrentPosition() - armSlideOffset);
                // Read the gamepad inputs
                Orientation pose = imu.getAngularOrientation();
                double yaw = pose.firstAngle * 60; // Calculate yaw from angles provided by the IMU
                double yawRadians = Math.toRadians(yaw - offset);
                double x = -(Math.sin(yawRadians) * gamepad1.left_stick_x + Math.cos(yawRadians) * gamepad1.left_stick_y);
                double y = Math.cos(yawRadians) * -gamepad1.left_stick_x + Math.sin(yawRadians) * gamepad1.left_stick_y;
                double rotation = -gamepad1.right_stick_x;

                if (gamepad1.left_bumper)
                    offset = yaw;

                double accelerator = 0.1f;
                accelerator += gamepad1.right_trigger;

                // Drive the robot
                frontLeft.setPower((x - y - rotation)*accelerator);
                frontRight.setPower((x + y + rotation)*accelerator);
                backLeft.setPower((x + y - rotation)*accelerator);
                backRight.setPower((x - y + rotation)*accelerator);

                armTarget += gamepad2.left_stick_y * 3;
                armSlideTarget += gamepad2.right_stick_y * 30;

//                armSlideTarget = Math.max(-350,armSlideTarget);

                handServoAngle += gamepad2.left_stick_x/50f;
                handWristServoAngle -= gamepad2.right_stick_x/50f;
                tiltServoAngle += gamepad2.dpad_right ? .01f :  gamepad2.dpad_left ? -.01f : 0;

                // Limit the servo range of motion
                handServoAngle = Math.min(0.8f, Math.max(0.3f, handServoAngle));
                handWristServoAngle = Math.min(0.8f, Math.max(0.3f, handWristServoAngle));
                tiltServoAngle = Math.min(0.8f, Math.max(0.3f, tiltServoAngle));
                // Calculate and drive the arm
                double armPowerTarget = (armTarget - armAngle) / 100f;
                double armSlidePowerTarget = (armSlideTarget - armSlideLength) / 100f;
                if (!gamepad2.a) {
                    armRotate.setPower(armPowerTarget);
                    armSlide.setPower(armSlidePowerTarget);
                } else {
                    armRotate.setPower(0);
                    armSlide.setPower(0);
                }
                handServo.setPosition(handServoAngle);
                handWristServo.setPosition(handWristServoAngle);
                tiltServo.setPosition(tiltServoAngle);

                if (gamepad2.y && !updatePressed) {
                    if (telemetryMode == TelemetryMode.ARM)
                        telemetryMode = TelemetryMode.ALL;
                    if (telemetryMode == TelemetryMode.MOVEMENT)
                        telemetryMode = TelemetryMode.ARM;
                    if (telemetryMode == TelemetryMode.ALL)
                        telemetryMode = TelemetryMode.MOVEMENT;
                }
                updatePressed = gamepad2.y;

//                Movement Telemetry
                telemetry.addData("Telemetry Mode", telemetryMode.name());
                if (telemetryMode.equals(TelemetryMode.ALL) || telemetryMode.equals(TelemetryMode.MOVEMENT)) {
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
                }

                // Arm Rotate Data
                if (telemetryMode.equals(TelemetryMode.ALL) || telemetryMode.equals(TelemetryMode.ARM)) {
                    telemetry.addData("armActualAngle", armRotate.getCurrentPosition());
                    telemetry.addData("armAngle", armAngle);
                    telemetry.addData("targetArmAngle", armTarget);
                    telemetry.addData("armAppliedPower", armPowerTarget);

                    // Slide Data
                    telemetry.addData("armActualAngle", armSlide.getCurrentPosition());
                    telemetry.addData("armAngle", armSlideLength);
                    telemetry.addData("targetArmAngle", armSlideTarget);
                    telemetry.addData("slideAppliedPower", armSlidePowerTarget);
                }

                telemetry.update();
            }
        }
    }
}
