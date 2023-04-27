package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_rotSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                                DoubleSupplier xSupplier,
                                DoubleSupplier rotSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_xSupplier = xSupplier;
        this.m_rotSupplier = rotSupplier;

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(
                m_xSupplier.getAsDouble(),
                m_rotSupplier.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0);
    }
}
