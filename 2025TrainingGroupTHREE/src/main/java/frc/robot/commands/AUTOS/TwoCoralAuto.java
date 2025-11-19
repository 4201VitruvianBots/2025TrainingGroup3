// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AUTOS;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoCoralAuto extends SequentialCommandGroup {
  /** Creates a new TwoCoralAuto. */
  public TwoCoralAuto(CommandSwerveDrivetrain swerveDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    try{
      PathPlannerPath path = PathPlannerPath.fromPathFile("ToReef1");

      var m_ToReef1 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("ToReef1");
      var m_collectCoral = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("collectCoral");
      var m_toReef2 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("ToReef2");

      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var starting_pose = path.getStartingHolonomicPose().orElseThrow();

      addCommands(
        new InstantCommand(() -> swerveDrive.resetPose(starting_pose)),
        m_ToReef1.andThen(() -> swerveDrive.setControl(stopRequest)),
        new WaitCommand(1), 
        m_collectCoral.andThen(() -> swerveDrive.setControl(stopRequest)),
        new WaitCommand(1), 
        m_toReef2.andThen(() -> swerveDrive.setControl(stopRequest)),
        new WaitCommand(1)
        );
    }


    catch (Exception e) {
      DriverStation.reportError("Failed to load path for TwoAlgaeRight", e.getStackTrace());
      addCommands(new WaitCommand(0));

  
    }
  }
}