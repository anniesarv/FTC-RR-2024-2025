package org.firstinspires.ftc.teamcode.utils;

//import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants;

public class Cache {
    public static boolean shouldUpdate(double lastValue, double currentValue, double tolerance) {
        return (Math.abs(currentValue - lastValue) >= tolerance ||
                (currentValue == 0.0 && lastValue != 0.0) ||
                (currentValue >= 1.0 && !(lastValue >= 1.0)) ||
                (currentValue <= -1.0 && !(lastValue <= -1.0)) ||
                Double.isNaN(lastValue));
    }

    public static boolean shouldUpdate(double lastValue, double currentValue) {
        return shouldUpdate(lastValue, currentValue);// DrivebaseConstants.Measurements.cachingTolerance);
    }
}
