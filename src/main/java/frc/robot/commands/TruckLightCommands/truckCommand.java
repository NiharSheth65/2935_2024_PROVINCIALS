// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TruckLightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TruckSubsystem;

public class truckCommand extends Command {
  /** Creates a new truckCommand. */

  private TruckSubsystem TRUCK_SUBSYSTEM; 
  private double truckBrightness; 
  
  
  public truckCommand(TruckSubsystem truckLight, double brightness) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.TRUCK_SUBSYSTEM = truckLight;   
    this.truckBrightness = brightness; 

    addRequirements(TRUCK_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TRUCK_SUBSYSTEM.setTruckBrightness(truckBrightness);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
