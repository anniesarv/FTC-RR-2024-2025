package org.firstinspires.ftc.teamcode.notinuse.elevator;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.notinuse.VerticalConstants;
import org.firstinspires.ftc.teamcode.utils.Cache;

public class Elevator extends SubsystemBase {

    private DcMotorEx motorElevator;
    private DigitalChannel limitElevator;

    private double currentPosition = 0.0;
    private double positionOffset = 0.0;
    private double lastSpeed = 0.0;
    private boolean atBottom = true;
    private boolean lastAtBottom = false;
    private double speed = 0.0;

    private final double UPPER_LIMIT = VerticalConstants.ElevatorPositions.UPPER_LIMIT;
    private final double LOWER_LIMIT = VerticalConstants.ElevatorPositions.LOWER_LIMIT;
    private final double TICKS_TO_INCHES = VerticalConstants.ElevatorConstants.TICKS_TO_INCHES;
    private final double KG = VerticalConstants.ElevatorCoefficients.KG;

    public Elevator(RobotHardware hardware) {
        this.motorElevator = (DcMotorEx) hardware.motorElevator;
        this.limitElevator = (DigitalChannel) hardware.limitElevator;
    }

    @Override
    public void periodic() {
        currentPosition = (motorElevator.getCurrentPosition() * TICKS_TO_INCHES) - positionOffset;

        atBottom = !limitElevator.getState();
        if (atBottom && !lastAtBottom) {
            positionOffset += currentPosition;
        }
        lastAtBottom = atBottom;
    }

    public void setRawSpeed(double speed) {
        double corrected = Math.max(-1.0, Math.min(speed, 1.0));
        if (Cache.shouldUpdate(lastSpeed, corrected)) {
            motorElevator.setPower(corrected);
            lastSpeed = corrected;
        }
    }

    public void setSpeed(double speed) {
        currentPosition = getPosition();

        if ((currentPosition < LOWER_LIMIT && speed <= 0.0) || (currentPosition > UPPER_LIMIT && speed > 0.0)) {
            setRawSpeed(0.0);
        } else {
            setRawSpeed(speed + KG);
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
}
