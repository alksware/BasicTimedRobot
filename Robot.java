// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.XboxController; 




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
  XboxController xbox = new XboxController(0);
  
  MotorControllerGroup leftGroup = new MotorControllerGroup(frontLeft, rearLeft);
  MotorControllerGroup rightGroup  = new MotorControllerGroup(frontRight, rearRight);

  DifferentialDrive motor = new DifferentialDrive(leftGroup,rightGroup);
  
  Compressor comp = new Compressor(PneumaticsModuleType.CTREPCM);
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0,1);

  Encoder encoder = new Encoder(0,1,true,EncodingType.k4X); 

  Timer timer = new Timer();

  AHRS navX = new AHRS(Port.kMXP);

  double setpoint = 0; 
  final double kP = 0.5;  
  final double kI = 0.5; 
  final double kD = 0.1;
  final double iLimit = 1;
  double errorSum = 0; 
  double lastTimestamp = 0; 
  double lastError = 0;
  final double kDrive = 1.0/128*6*Math.PI/12; 

  @Override
  public void robotInit() {
    encoder.reset(); 
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", encoder.get()*kDrive);
  }

  
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    encoder.reset();
    errorSum = 0;
    lastTimestamp = Timer.getFPGATimestamp();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    navX.getAngleAdjustment();

    if(xbox.getRawButton(1)){  
      setpoint = 10;
    }
    else if(xbox.getRawButton(2)){ 
      setpoint = 0;
    }

    //get sensor position
    double sensorPosition = encoder.get()*kDrive;
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    if(Math.abs(error)<iLimit){
      errorSum += error*dt;     //sor
    }
    double errorRate = (error- lastError) / dt; // sor
    
   
    double outputSpeed_ = kP*error*kI*errorSum;
    double outputSpeed = kP * error;

    frontLeft.set(outputSpeed);
    frontRight.set(outputSpeed);
    rearLeft.set(-outputSpeed);
    rearRight.set(-outputSpeed);

      lastTimestamp = Timer.getFPGATimestamp(); 
      lastError = error;
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
    
    if(joystick.getRawButton(3)){
      intake.set(1);
    }
    else if(joystick.getRawButtonReleased(3)){
      intake.set(0);
    }
    else if(joystick.getRawButton(4)){
      intake.set(-1);
    }
    else if(joystick.getRawButtonPressed(4)){
      intake.set(0);
    }

    if(joystick.getRawButton(2)){
      feeder.set(1);
    }
    else if(joystick.getRawButtonReleased(2)){
      feeder.set(0);
    }
    else if(joystick.getRawButton(6)){
      feeder.set(-1);
    }
    else if(joystick.getRawButtonReleased(6)){
      feeder.set(0);
    }
    
    else if(joystick.getThrottle() == -1){
      solenoid.set(Value.kForward);
    }
    else if(joystick.getThrottle() == 1 ){
      solenoid.set(Value.kReverse);
    }
    
    if(joystick.getRawButton(11)){
      comp.enableDigital();
    }
    else if(joystick.getRawButtonReleased(12)){
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
