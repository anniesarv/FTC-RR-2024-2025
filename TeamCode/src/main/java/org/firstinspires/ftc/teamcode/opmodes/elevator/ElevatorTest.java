package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorCommand;
import java.util.function.DoubleSupplier;

@TeleOp
public class ElevatorTest extends LinearOpMode {
    private RobotHardware robotHardware;
    private Elevator elevator;
    //private ElevatorCommand elevatorCommand;

    @Override
    public void runOpMode() {
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap);
        elevator = new Elevator(robotHardware);

        // doublesupplier for gamepad im confused
        DoubleSupplier elevatorSpeedSupplier = () -> -gamepad1.left_stick_y;  // Negative for correct direction control

        //elevatorCommand = new ElevatorCommand(elevator, elevatorSpeedSupplier);

        //CommandScheduler.getInstance().schedule(elevatorCommand);

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            double currentPosition = elevator.getPosition();
            telemetry.addData("Elevator Encoder Ticks", currentPosition);
            telemetry.update();
        }
    }
}
