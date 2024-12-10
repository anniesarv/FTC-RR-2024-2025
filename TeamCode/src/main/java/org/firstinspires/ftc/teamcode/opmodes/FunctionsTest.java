package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.TeleOpFunctions;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeWrist;

@TeleOp
public class FunctionsTest extends CommandOpMode {

    private RobotHardware robotHardware;
    private IntakeWrist intakeWrist;
    private OuttakeWrist outtakeWrist;

    @Override
    public void initialize() {
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap);

        intakeWrist = new IntakeWrist(robotHardware);
        outtakeWrist = new OuttakeWrist(robotHardware);

        CommandScheduler.getInstance().schedule(
                new TeleOpFunctions(intakeWrist, outtakeWrist, gamepad1)
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
