package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.notinuse.DriveTrain;
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveCommand;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp
public class Drive extends OpMode {

    private RobotHardware robotHardware;
    private DriveTrain drivetrain;
    private GamepadEx driver1;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap);
        drivetrain = new DriveTrain(robotHardware);

        driver1 = new GamepadEx(gamepad1);

        drivetrain.setDefaultCommand(new DriveCommand(
                drivetrain,
                () -> -driver1.getLeftX(),
                () -> -driver1.getLeftY(),
                () -> driver1.getRightX()
        ));
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}
