// DRIVE SUBSYSTEM

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains the code necessary to operate a robot with arcade drive
 * using only the left joystick.
 */
public class RobotChattyTest extends TimedRobot {
  private final DifferentialDrive m_robotDrive;
  private final Joystick m_leftStick;

  private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // Invert one motor so both sides move forward with positive input
    m_rightMotor.setInverted(true);

    m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_leftStick = new Joystick(0); // Only one joystick used

    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
  }

  @Override
  public void teleopPeriodic() {
    // Use Y axis for forward/backward and X axis for turning
    double forward = -m_leftStick.getY();  // invert for forward
    double turn = m_leftStick.getX();      // turning left/right

    m_robotDrive.arcadeDrive(forward, turn);
  }
}
