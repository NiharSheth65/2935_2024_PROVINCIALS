// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** Creates a new IntakeSubsystem. */
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

    // CREATING A CONTAINER FOR PRINTING DRIVE THINGS TO DASHBOARD 
  private final String INTAKE_PREFIX = "SmartDashboard/Intake"; 

  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorId, MotorType.kBrushless); 
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder(); 

  
  public IntakeSubsystem() {
    // RESTORE SETTINGS 
    intakeMotor.restoreFactoryDefaults(); 
    // INVERT 
    intakeMotor.setInverted(true);
    // SET POSITION TO 0 
    intakeEncoder.setPosition(0);
    // LOAD CONFIG 
    intakeMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     // PRINT INTAKE POSITION
     SmartDashboard.putNumber(INTAKE_PREFIX + "intake encoder", getIntakeEncoderPosition()); 
     
     // PRINT INTAKE VELOCITY   
     SmartDashboard.putNumber(INTAKE_PREFIX + "intake velocity", getIntakeEncoderVelocity()); 
 
     // PRINT MOTOR CURRENT USAGE 
     SmartDashboard.putNumber(INTAKE_PREFIX + "intake current", getIntakeEncoderCurrent()); 

  }

  // SET TO BRAKE MODE 
  public void setBrakeMode(){
    intakeMotor.setIdleMode(IdleMode.kBrake); 
  }

  // SET TO COAST MODE 
  public void setCoastMode(){
    intakeMotor.setIdleMode(IdleMode.kCoast); 
  }

  // RESET ENCODER
  public void resetEncoders(){
    intakeEncoder.setPosition(0);  
  }

  public double getIntakeEncoderPosition(){
    return intakeEncoder.getPosition(); 
  }

  public double getIntakeEncoderVelocity(){
    return intakeEncoder.getVelocity(); 
  }

  public double getIntakeEncoderCurrent(){
    return intakeMotor.getOutputCurrent(); 
  }


  public boolean intakeHasIntaked(){
    
    if(getIntakeEncoderCurrent() > IntakeConstants.intakeHasIntakedThreshhold){
      return true; 
    }

    else{
      return false; 
    }
  }

  // SET INTAKE PID VALUE 
  public void setLeftIntakePIDF(double p, double i, double d, double f){
    intakeMotor.getPIDController().setP(p); 
    intakeMotor.getPIDController().setI(i); 
    intakeMotor.getPIDController().setD(d); 
    intakeMotor.getPIDController().setFF(f); 
  }

  // SET INTAKE CONSTRAINTS
  public void setIntakeOutPutConstraints(double min, double max){
    intakeMotor.getPIDController().setOutputRange(min, max); 
  }

  // SET VELOCITY MODE 
  public void setIntakeVelocityMode(){
    intakeMotor.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  // SET POWER MODE 
  public void setIntakePowerMode(){
    intakeMotor.getPIDController().setReference(0, ControlType.kVoltage); 
  }

  // SET VELOCITY MODE 
  public void setVelocity(double intakeVelocity){
    intakeMotor.getPIDController().setReference(intakeVelocity, ControlType.kVelocity); 
  }

  // SET INTAKE 
  public void setIntake(double intakeSpeed){
    intakeMotor.set(intakeSpeed);
  }

  // STOP INTAKE 
  public void stop(){
    intakeMotor.stopMotor();
  }
}
