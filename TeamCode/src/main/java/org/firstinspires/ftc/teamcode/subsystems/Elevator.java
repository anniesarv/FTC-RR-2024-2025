package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.constants.DeviceIDs;
import org.firstinspires.ftc.teamcode.constants.VerticalConstants;
import org.firstinspires.ftc.teamcode.utils.Cache;

public class VerticalArm extends SubsystemBase {

    private DcMotorEx motorLeft;
    private DcMotorEx motorRight;
    private TouchSensor limit;

    private double currentPosition = 0.0;
    private double positionOffset = 0.0;
    private double lastSpeed = 0.0;
    private boolean atBottom = true;
    private boolean lastAtBottom = false;
    private double currentLeft = 0.0;
    private double currentRight = 0.0;
    private double speed = 0.0;

    private final VerticalConstants.ElevatorPositions positions = VerticalConstants.ElevatorPositions;
    private final VerticalConstants.ElevatorCoefficients coefficients = VerticalConstants.ElevatorCoefficients;
    private final VerticalConstants.ElevatorConstants constants = VerticalConstants.ElevatorConstants;

    public VerticalArm(HardwareMap hardwareMap) {
        motorLeft = hardwareMap.get(DcMotorEx.class, DeviceIDs.ELEVATOR_LEFT);
        motorRight = hardwareMap.get(DcMotorEx.class, DeviceIDs.ELEVATOR_RIGHT);
        limit = hardwareMap.get(TouchSensor.class, DeviceIDs.VERTICAL_LIMIT);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        currentPosition = (motorLeft.getCurrentPosition() * constants.TICKS_TO_INCHES) - positionOffset;

        atBottom = limit.isPressed();
        if (atBottom && !lastAtBottom) {
            positionOffset += currentPosition;
        }
        lastAtBottom = atBottom;
    }

    public void setRawSpeed(double speed) {
        double corrected = Math.max(-1.0, Math.min(speed, 1.0));
        if (Cache.shouldUpdate(lastSpeed, corrected)) {
            motorLeft.setPower(corrected);
            motorRight.setPower(corrected);
            lastSpeed = corrected;
        }
    }

    public void setSpeed(double speed) {
        currentPosition = getPosition();

        if ((currentPosition < positions.LOWER_LIMIT && speed <= 0.0) || (currentPosition > positions.UPPER_LIMIT && speed > 0.0)) {
            setRawSpeed(0.0);
        } else {
            setRawSpeed(speed + coefficients.KG);
        }
    }

    public double getPosition() {
        return currentPosition;
    }

    public boolean atBottom() {
        return atBottom;
    }

    public double getOffset() {
        return positionOffset;
    }

    public double getSpeed() {
        return speed;
    }

    public double getCurrentLeft() {
        return currentLeft;
    }

    public double getCurrentRight() {
        return currentRight;
    }

    public double getCurrent() {
        return currentRight + currentLeft;
    }
}
