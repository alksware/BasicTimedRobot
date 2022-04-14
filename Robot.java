// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  Victor frontLeft = new Victor(1);
  Victor frontRight = new Victor(0);
  Victor rearLeft = new Victor(3);
  Victor rearRight = new Victor(2);
  Victor intake = new Victor(4);
  Victor feeder = new Victor(7);
  Victor frontshooter = new Victor(5);
  Victor rearShooter = new Victor(6);

  Joystick joystick = new Joystick(0);
  
  MotorControllerGroup leftGroup = new MotorControllerGroup(frontLeft, rearLeft);
  MotorControllerGroup rightGroup  = new MotorControllerGroup(frontRight, rearRight);

  DifferentialDrive motor = new DifferentialDrive(leftGroup,rightGroup);
  
  Compressor comp = new Compressor(PneumaticsModuleType.CTREPCM);
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0,1);

  Timer timer = new Timer();

  @Override
  public void robotInit() {
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if(timer.get()<3){
      leftGroup.set(0.20);
      rightGroup.set(0.20);
    }
    else if(timer.get()>3 & timer.get()<5){
      comp.enableDigital();
      solenoid.set(Value.kReverse);
    }
    else if(timer.get()>5 & timer.get()<7){
      intake.set(1);
    }
    else if(timer.get()>8 & timer.get()<10){
      intake.set(0);
      feeder.set(-1);
    }
    else if(timer.get()>11 & timer.get()<13){
      feeder.set(0);
      frontshooter.set(-1);
      rearShooter.set(-1);
    }
    else if(timer.get()>13 & timer.get()<16){
      frontshooter.set(0);
      rearShooter.set(0);
      leftGroup.set(-0.20);
      rightGroup.set(-0.20);
    }
    else if(timer.get()>16 & timer.get()<19){
      comp.disable();
      solenoid.set(Value.kForward);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    motor.arcadeDrive(joystick.getRawAxis(1),joystick.getRawAxis(2));

    if(joystick.getRawButton(1)){
      frontshooter.set(1);
      rearShooter.set(1);
    }
    else if(joystick.getRawButtonReleased(1)){
      frontshooter.set(0);
      rearShooter.set(0);
    }
    else if(joystick.getRawButton(5)){
      frontshooter.set(-1);
      rearShooter.set(-1);
    }
    else if(joystick.getRawButtonReleased(5)){
      frontshooter.set(0);
      rearShooter.set(0);
    }
    
    if(joystick.getRawButton(2)){
      intake.set(1);
    }
    else if(joystick.getRawButtonReleased(2)){
      intake.set(0);
    }
    else if(joystick.getRawButton(3)){
      intake.set(-1);
    }
    else if(joystick.getRawButtonPressed(3)){
      intake.set(0);
    }

    if(joystick.getRawButton(4)){
      feeder.set(1);
    }
    else if(joystick.getRawButtonReleased(4)){
      feeder.set(0);
    }
    else if(joystick.getRawButton(6)){
      feeder.set(-1);
    }
    else if(joystick.getRawButtonReleased(6)){
      feeder.set(0);
    }

    if(joystick.getRawButton(12)){
      solenoid.set(Value.kForward);
    }
    else if(joystick.getRawButton(11)){
      solenoid.set(Value.kReverse);
    }
    if(joystick.getRawButton(8)){
      comp.enableDigital();
    }
    else if(joystick.getRawButtonReleased(9)){
      comp.disable();
    }
    
  }
    
    

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
