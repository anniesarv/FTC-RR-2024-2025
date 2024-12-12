package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    public DcMotor motorLF, motorRF, motorLR, motorRR, motorVA, motorHA;
    public Servo horizontalWrist, verticalWrist, microServo;
    public CRServo intakeLeft, intakeRight;


    // sensors
    public DigitalChannel limitVA, limitHA;

    // hardware map
    private HardwareMap hwMap;

    // Constructor
    public RobotHardware() {
        // leave empty for now ?
    }


    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        // Initialize motors
        motorLF = hwMap.get(DcMotor.class, "motorLF");
        motorRF = hwMap.get(DcMotor.class, "motorRF");
        motorLR = hwMap.get(DcMotor.class, "motorLR");
        motorRR = hwMap.get(DcMotor.class, "motorRR");
        motorVA = hwMap.get(DcMotor.class, "motorVA");
        motorHA = hwMap.get(DcMotor.class, "motorHA");

        // Set motor directions
        motorLF.setDirection(DcMotor.Direction.FORWARD);
        motorLR.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorRR.setDirection(DcMotor.Direction.FORWARD);

        // set motor modes and zero power behavior
        DcMotor[] driveMotors = {motorLF, motorRF, motorLR, motorRR, motorVA, motorHA};
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
        horizontalWrist.setPosition(0.5);
        verticalWrist.setPosition(0.5);

        // Initialize sensors
        limitVA = hwMap.get(DigitalChannel.class, "limitVA");
        limitVA.setMode(DigitalChannel.Mode.INPUT);

        limitHA= hwMap.get(DigitalChannel.class, "limitHA");
        limitHA.setMode(DigitalChannel.Mode.INPUT);




    }
}
