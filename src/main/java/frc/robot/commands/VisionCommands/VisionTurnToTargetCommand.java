// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionCommands;

import java.lang.annotation.Target;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.StatusVariables;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionTurnToTargetCommand extends Command {

  private VisionSubsystem VISION_SUBSYSTEM; 
  private DriveSubsystem DRIVE_SUBSYSTEM; 

  private PIDController turnVisionPID; 
  

  private boolean endCommand; 

  private int setPipelineNumber; 

  private double measuredValue; 
  private double turnSpeed;

  private double initTime; 

  private double tSlew = 2; 
  private SlewRateLimiter turn_Limiter = new SlewRateLimiter(tSlew); 

  private double commandInitYaw; 
  private double commandFinalYaw; 

  /** Creates a new VisionTurnToTargetCommand. */
  public VisionTurnToTargetCommand(DriveSubsystem drive, VisionSubsystem vision, int pipeline, boolean end) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.VISION_SUBSYSTEM = vision; 

    this.turnVisionPID = new PIDController(VisionConstants.visionTurnKp, VisionConstants.visionTurnKi, VisionConstants.visionTurnKd); 
    this.endCommand = end; 
    this.setPipelineNumber = pipeline;

    addRequirements(DRIVE_SUBSYSTEM);
    addRequirements(VISION_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnVisionPID.reset();
    VISION_SUBSYSTEM.setPipeline(setPipelineNumber);
    initTime = System.currentTimeMillis(); 
    VisionConstants.adjustEndGyroAngle = 0; 
    VisionConstants.adjust = 0; 
    commandInitYaw = DRIVE_SUBSYSTEM.getYaw(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(VISION_SUBSYSTEM.limelightTargetSeen()){
      measuredValue = VISION_SUBSYSTEM.getTx(); 
    }

    else{
      measuredValue = 0; 
    }

    turnSpeed = turnVisionPID.calculate(measuredValue, 0); 

    if(turnSpeed > VisionConstants.limelightTurnSpeedLimit){
      turnSpeed = VisionConstants.limelightTurnSpeedLimit; 
    }

    else if(turnSpeed < -VisionConstants.limelightTurnSpeedLimit){
      turnSpeed = -VisionConstants.limelightTurnSpeedLimit; 
    }

    DRIVE_SUBSYSTEM.setTank(turn_Limiter.calculate(turnSpeed), -turn_Limiter.calculate(turnSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    commandFinalYaw = DRIVE_SUBSYSTEM.getYaw(); 
    VisionConstants.adjustEndGyroAngle = DRIVE_SUBSYSTEM.getYaw(); 
    VisionConstants.adjust = VisionConstants.adjustEndGyroAngle - VisionConstants.driveStartGyroAngle;     
    DRIVE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(endCommand == true){
      return true; 
    }
    
    else if(Math.abs(VISION_SUBSYSTEM.getTx()) < VisionConstants.fineAlignmentTolerance){
      return true; 

    }
    
    else if(Math.abs(System.currentTimeMillis() - initTime) > VisionConstants.limelightCommandTimeout){
      return true;
    }

    else{
      return false; 
    }

  }
}
