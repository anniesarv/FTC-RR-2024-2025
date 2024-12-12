
package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import java.util.function.DoubleSupplier;

public class DriveTrainCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrain;
    private final DoubleSupplier m_xSupplier, m_ySupplier, m_rotationSupplier;

    public DriveTrainCommand(DriveTrain drivetrain,
                        DoubleSupplier xSupplier,
                        DoubleSupplier ySupplier,
                        DoubleSupplier rotationSupplier) {
        m_drivetrain = drivetrain;
        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;
        m_rotationSupplier = rotationSupplier;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        // Pass joystick inputs to the drivetrain subsystem
        m_drivetrain.drive(m_xSupplier.getAsDouble(),
                m_ySupplier.getAsDouble(),
                m_rotationSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain motors when the command ends
        m_drivetrain.stopMotors();
    }

    @Override
    public boolean isFinished() {
        // This command runs until explicitly stopped
        return false;
    }
}