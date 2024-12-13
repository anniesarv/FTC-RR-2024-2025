package org.firstinspires.ftc.teamcode.opmodes.elevator;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class ElevatorEncoderTest extends OpMode {

    private RobotHardware robot = new RobotHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        int encoderTicks = robot.motorElevator.getCurrentPosition();

        telemetry.addData("Motor Elevator Encoder Ticks", encoderTicks);
        telemetry.update();
    }
}


//ended up at 882 encoder tics for 42.25 inches
//20.875739645 tics per inch