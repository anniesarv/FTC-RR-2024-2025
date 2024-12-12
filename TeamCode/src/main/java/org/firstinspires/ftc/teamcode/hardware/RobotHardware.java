package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    public DcMotor motorLF, motorRF, motorLR, motorRR, motorElevator, motorExtension;
    public Servo horizontalWrist, verticalWrist, microServo;
    public CRServo intakeLeft, intakeRight;

    // sensors
    public DigitalChannel limitElevator, limitExtension;
    public BNO055IMU imu;

    // hardware map
    private HardwareMap hwMap;

    // Constructor
    public RobotHardware() {
        // leave empty for now
    }

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        // Initialize motors
        motorLF = hwMap.get(DcMotor.class, "motorLF");
        motorRF = hwMap.get(DcMotor.class, "motorRF");
        motorLR = hwMap.get(DcMotor.class, "motorLR");
        motorRR = hwMap.get(DcMotor.class, "motorRR");
        motorElevator = hwMap.get(DcMotor.class, "motorElevator");
        motorExtension = hwMap.get(DcMotor.class, "motorExtension");

        // Set motor directions
        motorLF.setDirection(DcMotor.Direction.FORWARD);
        motorLR.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setDirection(DcMotor.Direction.REVERSE);

        // set motor modes and zero power behavior
        DcMotor[] driveMotors = {motorLF, motorRF, motorLR, motorRR, motorElevator, motorExtension};
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // initialize servos
        horizontalWrist = hwMap.get(Servo.class, "horizontalWrist");
        verticalWrist = hwMap.get(Servo.class, "verticalWrist");
        microServo = hwMap.get(Servo.class, "microServo");
        intakeLeft = hwMap.get(CRServo.class, "intakeLeft");
        intakeRight = hwMap.get(CRServo.class, "intakeRight");

        // set default servo positions
        horizontalWrist.setPosition(1);
        verticalWrist.setPosition(0);

        // Initialize sensors
        limitElevator = hwMap.get(DigitalChannel.class, "limitElevator");
        limitElevator.setMode(DigitalChannel.Mode.INPUT);

        limitExtension = hwMap.get(DigitalChannel.class, "limitExtension");
        limitExtension.setMode(DigitalChannel.Mode.INPUT);

        // Initialize and configure IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        initializeImu();
    }

    public void initializeImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }
}