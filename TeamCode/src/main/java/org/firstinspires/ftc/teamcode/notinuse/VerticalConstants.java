package org.firstinspires.ftc.teamcode.notinuse;

import com.acmerobotics.dashboard.config.Config;

public class VerticalConstants {

    @Config
    public static class ElevatorCoefficients {
        public static double KS = 0.0;
        public static double KV = 0.0;
        public static double KA = 0.0;

        public static double KP = 0.21; // 0.30
        public static double KI = 0.0;
        public static double KD = 0.0;
        public static double KF = 0.0;

        public static double KG = 0.0; // 0.042
        public static double KE = 0.0027;
    }

    @Config
    public static class ElevatorPositions {
        public static double UPPER_LIMIT = 28.1; // 28.2
        public static double LOWER_LIMIT = 0.0;  // 28.2
        public static double TOP = 27.75;       // 28.2
        public static double BOTTOM = 0.20;
        public static double ARM = 3.5;
        public static double ARM_TARGET = 3.75;
        public static double SPECIMEN_PLACE = 13.5;
        public static double CLIMB_ONE = 20.0;
        public static double CLIMB_TWO = 8.0;
        public static double CLIMB_THREE = 26.5;
    }

    @Config
    public static class ElevatorConstants {
        //public static double TICKS_TO_INCHES = 0.04790249433;
        public static double TICKS_TO_INCHES = 20.875739645;
        // public static double POSITION_TOLERANCE = 0.25;
        public static double POSITION_TOLERANCE = 0.5;
        public static double VELOCITY_TOLERANCE = 5.0;
    }


    /*
    @Config
    public static class VerticalArmPositions {
        public static double INTAKE = 0.0;
        public static double SAMPLE = 0.6;
        public static double SPECIMEN = 0.93;
        public static double AUTO_START = 0.25;
    }

    @Config
    public static class VerticalArmConstants {
        public static double unitsToSeconds = 0.4;
        public static double intakeToSpecimen = 0.5;
        public static double specimenToIntake = 0.8;

        public static double intakeToSample = 0.3;
        public static double sampleToIntake = 0.020;
    }

    @Config
    public static class DepositPositions {
        public static double IN = 0.0;
        public static double MID = 0.08;
        public static double OUT = 1.00;
    }

    @Config
    public static class VerticalWristPositions {
        public static double INTAKE = 0.02; // home
        public static double SAMPLE = 0.45; // straight
        public static double SPECIMEN_PICKUP = 0.65; // wall
        public static double SPECIMEN_PLACE = 0.935; // specimen place
    }

     */

    /**
     * vertical arm
     * angle 210 units
     * pwm power 74.9%
     * sensitivity ultra high
     *
     * vertical wrist
     * angle 135 units
     * pwm power 74.9%
     *
     * deposit
     * angle 50 units
     * pwm power 94.1%
     * sensitivity high
     */
}
