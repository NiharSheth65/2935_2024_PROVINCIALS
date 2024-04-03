// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

  private final String CLIMB_PREFIX = "SmartDashboard/Climb"; 

  private CANSparkMax climbMotor = new CANSparkMax(ClimbConstants.climbMotorID, MotorType.kBrushless); 
  private RelativeEncoder climbEncoder = climbMotor.getEncoder(); 


  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    // RESTORE SETTINGS 
    climbMotor.restoreFactoryDefaults(); 

    // DONT INVERT 
    climbMotor.setInverted(false);

    // SET INITIAL POSITION TO 0 
    climbEncoder.setPosition(0);

    // SET CLIMB TO BRAKE MODE 
    setBrakeMode();

    // SAVE SETTINGS BY BURNING FLASH 
    climbMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // PRINT CLIMB POSITION
     SmartDashboard.putNumber(CLIMB_PREFIX + "Climb encoder", getClimbEncoderPosition()); 
     
     // PRINT Climb VELOCITY   
     SmartDashboard.putNumber(CLIMB_PREFIX + "Climb velocity", getClimbEncoderVelocity()); 
 
     // PRINT MOTOR CURRENT USAGE 
     SmartDashboard.putNumber(CLIMB_PREFIX + "Climb current", getClimbEncoderCurrent()); 
  }

    // GET CLIMB MOTOR ENCODER POSITION  
  public double getClimbEncoderPosition(){
    return climbEncoder.getPosition(); 
  }

  // GET CLIMB MOTOR ENCODER CURRENT 
  public double getClimbEncoderCurrent(){
    return climbMotor.getOutputCurrent(); 
  }

  // GET CLIMB MOTOR VELOCITY 
  public double getClimbEncoderVelocity(){
    return climbEncoder.getVelocity(); 
  }

  // SET CLIMB MODE TO BRAKE 
  public void setBrakeMode(){
    climbMotor.setIdleMode(IdleMode.kBrake); 
  }

  
  // SET CLIMB MODE TO COAST 
  public void setCoastMode(){
    climbMotor.setIdleMode(IdleMode.kCoast); 
  }

  // SET CLIMB MODE TO BRAKE 
  public void resetEncoders(){
    climbEncoder.setPosition(0);  
  }

  // SET CLIMB PIDS 
  public void setClimbPIDF(double p, double i, double d, double f){
    climbMotor.getPIDController().setP(p); 
    climbMotor.getPIDController().setI(i); 
    climbMotor.getPIDController().setD(d); 
    climbMotor.getPIDController().setFF(f); 
  }


  // SET CLIMB CONSTRAINTS
  public void setClimbOutPutConstraints(double min, double max){
    climbMotor.getPIDController().setOutputRange(min, max); 
  }

  // SET CLIMB VELOCITY MODE 
  public void setClimbVelocityMode(){
    climbMotor.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  // SET CLIMB POWER MODE 
  public void setClimbPowerMode(){
    climbMotor.getPIDController().setReference(0, ControlType.kVoltage); 
  }


  // SET CLIMB TO VELOCITY MODE 
  public void setClimbVelocity(double climbVelocity){
    climbMotor.getPIDController().setReference(climbVelocity, ControlType.kVelocity); 
  }

  // SET CLIMB IN POWER 
  public void setClimb(double climbSpeed){
    climbMotor.set(climbSpeed);
  }

  // SET CLIMB TO STOP 
  public void stop(){
    climbMotor.stopMotor();
  }

}
