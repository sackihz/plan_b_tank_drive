// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import javax.sql.rowset.spi.TransactionalWriter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick = new Joystick(1);
  private Joystick m_rightStick = new Joystick(2);
  //private XboxController x_box_controller = new XboxController(0);
  private SlewRateLimiter slew_limiter = new SlewRateLimiter(12);


  private final WPI_TalonSRX m_leftMotor = new WPI_TalonSRX(0); 
  private final WPI_TalonSRX m_rightMotor = new WPI_TalonSRX(2);

  private final WPI_TalonSRX m_leftMotor_2 = new WPI_TalonSRX(1);
  private final WPI_TalonSRX m_rightMotor_2 = new WPI_TalonSRX(3);

  private final MotorControllerGroup left = new MotorControllerGroup(m_leftMotor, m_leftMotor_2);
  private final MotorControllerGroup right = new MotorControllerGroup(m_rightMotor, m_rightMotor_2);

 
  private final PWMVictorSPX intake_motor  = new PWMVictorSPX(1); 
  private final PWMVictorSPX transfer_motor  = new PWMVictorSPX(2); 
  private final PWMVictorSPX shooter_motor  = new PWMVictorSPX(0);

  // intake right button 2 
  // shooter left toggle 1
  

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    left.setInverted(true);

    m_myRobot = new DifferentialDrive(right, left);
    m_leftStick = new Joystick(1);
    m_rightStick = new Joystick(2);
  }

  boolean shooter_toggle = false;

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
    
    if(m_leftStick.getRawButton(1)) {
      shooter_toggle = !shooter_toggle;
    }
    boolean actuate = m_rightStick.getRawButton(1);
    boolean override = m_rightStick.getRawButton(5);

    if (actuate == true && shooter_toggle == false) 
    {
      intake_motor.setVoltage(8);
    }
    if (actuate == true && shooter_toggle == true) 
    {
      transfer_motor.setVoltage(3);
    }
    if (override == true)
    {
      intake_motor.setVoltage(0);
    }

    if (shooter_toggle == true) 
    {
      shooter_motor.setVoltage(slew_limiter.calculate(11.25));
    }
    else
    {
      shooter_motor.setVoltage(slew_limiter.calculate(0));
    }



    if (m_rightStick.getRawButton(3) == true)
    {
      if (actuate == true && shooter_toggle == false) 
      {
        intake_motor.setVoltage(-8);
      }
      if (actuate == true && shooter_toggle == true) 
      {
        transfer_motor.setVoltage(-3);
      }
  
      if (shooter_toggle == true) 
      {
        shooter_motor.setVoltage(slew_limiter.calculate(-11.25));
      }
      else
      {
        shooter_motor.setVoltage(slew_limiter.calculate(0));
      }
      
    }
}
}