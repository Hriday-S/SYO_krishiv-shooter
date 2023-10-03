package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_leftSupplier;
    private final DoubleSupplier m_rightSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                                DoubleSupplier leftSupplier,
                                DoubleSupplier rightSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_leftSupplier = leftSupplier;
        this.m_rightSupplier = rightSupplier;

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(
                m_leftSupplier.getAsDouble(),
                m_rightSupplier.getAsDouble()*0.67
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0);
    }
}
