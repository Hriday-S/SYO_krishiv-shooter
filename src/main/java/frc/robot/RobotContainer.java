package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Commands.DefaultDriveCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class RobotContainer {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private Joystick m_stick;

    public RobotContainer() {
        m_drivetrainSubsystem = new DrivetrainSubsystem();

        m_stick = new Joystick(0);

        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
                m_drivetrainSubsystem, 
                () -> MathUtil.applyDeadband(-m_stick.getRawAxis(1), 0.05), 
                () -> MathUtil.applyDeadband(m_stick.getRawAxis(4), 0.05) * 0.25
        ));

        configureButtons();
    }

    private void configureButtons() {}
}
