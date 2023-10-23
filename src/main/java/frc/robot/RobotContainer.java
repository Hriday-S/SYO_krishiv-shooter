package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Commands.DefaultDriveCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class RobotContainer {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private Joystick m_stick;
    private Joystick m_controller;

    private int m_dpad;

    private DoubleSupplier m_leftSupplier = () -> 1;
    private DoubleSupplier m_rightSupplier = () -> 1;

    public RobotContainer() {
        m_drivetrainSubsystem = new DrivetrainSubsystem();
        m_stick = new Joystick(0);//joystick
        m_controller = new Joystick(1);//controller
        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
                m_drivetrainSubsystem, 
                () -> MathUtil.applyDeadband(m_controller.getRawAxis(3), 0.05), 
                () -> MathUtil.applyDeadband(m_controller.getRawAxis(1), 0.05)
        ));     
        configureButtons();
    }
    
  public SequentialCommandGroup autonomousCommands()
  {
    return new SequentialCommandGroup(new DefaultDriveCommand(m_drivetrainSubsystem, m_leftSupplier, m_rightSupplier));
  }
    private void configureButtons() {}
}
