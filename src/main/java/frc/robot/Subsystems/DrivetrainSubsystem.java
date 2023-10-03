package frc.robot.Subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.annotation.Inherited;

public class DrivetrainSubsystem extends SubsystemBase {
    private DifferentialDrive m_myRobot;

    private CANSparkMax m_frontleftMotor;
    private CANSparkMax m_rearleftMotor;
    private CANSparkMax m_frontrightMotor;
    private CANSparkMax m_rearrightMotor;
    
    private MotorControllerGroup m_leftMotors;
    private MotorControllerGroup m_rightMotors;

    private double m_xSpeed;
    private double m_ySpeed;

    private double error;


    public DrivetrainSubsystem() {
        m_frontleftMotor = new CANSparkMax(Constants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushed);
        m_rearleftMotor = new CANSparkMax(Constants.REAR_LEFT_MOTOR_PORT, MotorType.kBrushed);
        m_frontrightMotor = new CANSparkMax(Constants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushed);
        m_rearrightMotor = new CANSparkMax(Constants.REAR_RIGHT_MOTOR_PORT, MotorType.kBrushed);

        m_frontleftMotor.setIdleMode(IdleMode.kBrake);
        m_rearleftMotor.setIdleMode(IdleMode.kBrake);
        m_frontrightMotor.setIdleMode(IdleMode.kBrake);
        m_rearrightMotor.setIdleMode(IdleMode.kBrake);

        m_leftMotors = new MotorControllerGroup(m_frontleftMotor, m_rearleftMotor);
        m_rightMotors = new MotorControllerGroup(m_frontrightMotor, m_rearrightMotor);
        
        m_leftMotors.setInverted(true);

        m_myRobot = new DifferentialDrive(m_leftMotors, m_rightMotors);



    }
    
    

    public void drive(double xSpeed, double ySpeed) {
        error = 0;

        m_xSpeed = xSpeed - error;
        m_ySpeed = ySpeed + error;
        SmartDashboard.putNumber("error", error);
    }

    @Override
    public void periodic() {
        m_myRobot.tankDrive(m_xSpeed, m_ySpeed);
    }
}
