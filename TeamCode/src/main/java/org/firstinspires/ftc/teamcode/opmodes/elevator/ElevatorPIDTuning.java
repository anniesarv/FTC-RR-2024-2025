package org.firstinspires.ftc.teamcode.opmodes.elevator;

import com.arcrobotics.ftclib.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorPIDCommand;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.constants.VerticalConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ElevatorPIDTuning extends LinearOpMode {

    private Elevator elevator;
    private double setPointInches = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap);

        elevator = new Elevator(robotHardware);

        telemetry.addLine("Elevator PID Tuning");


        telemetry.update();

        // um i don't know
        double setPointTicks = setPointInches * (1.0 / VerticalConstants.ElevatorConstants.TICKS_TO_INCHES);

        // CREATE COMMAND OUTSIDE OF LOOP
        ElevatorPIDCommand elevatorCommand = new ElevatorPIDCommand(elevator, setPointTicks);

        waitForStart();

        while (opModeIsActive()) {
            // command is scheduled when a is pressed
            if (gamepad1.a) {
                CommandScheduler.getInstance().schedule(elevatorCommand);
            }

            // Run the scheduler
            CommandScheduler.getInstance().run();

            // Telemetry updates
            telemetry.addData("Set Point (inches)", setPointInches);
            telemetry.addData("Set Point (ticks)", setPointTicks);
            telemetry.addData("Command Scheduled", elevatorCommand.isScheduled());
            telemetry.addData("Elevator Position", elevator.getPosition());
            telemetry.update();

            // delay to prevent CPU overload ? what is this
            sleep(20);
        }
    }
}