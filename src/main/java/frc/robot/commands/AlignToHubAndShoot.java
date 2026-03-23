package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsytem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

public class AlignToHubAndShoot extends Command {
  private final SwerveSubsystem swerve;
  private final Vision vision;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final HopperSubsytem hopper;

  private final DoubleSupplier xInput;
  private final DoubleSupplier yInput;
  private Command armOscillateCommand;

  private int lockedTagId = -1;
  private Translation2d hubCenterField = null;
  private boolean currentlySeeingHubTag = false;

  public AlignToHubAndShoot(
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      HopperSubsytem hopper,
      DoubleSupplier xInput,
      DoubleSupplier yInput) {
    this.swerve = swerve;
    this.vision = swerve.getVision();
    this.shooter = shooter;
    this.indexer = indexer;
    this.hopper = hopper;
    this.xInput = xInput;
    this.yInput = yInput;

    addRequirements(swerve, shooter, indexer, hopper);
  }

  @Override
  public void initialize() {
    lockedTagId = -1;
    currentlySeeingHubTag = false;
    hubCenterField = vision.getHubCenterFieldPositionFromLayout(Constants.HUB_TAG_IDS).orElse(null);
  }

  @Override
  public void execute() {
    Optional<Integer> visibleTagOpt = vision.getBestVisibleHubTag(Constants.HUB_TAG_IDS);
    currentlySeeingHubTag = visibleTagOpt.isPresent();

    if (visibleTagOpt.isPresent()) {
      lockedTagId = visibleTagOpt.get();
    }

    if (lockedTagId != -1) {
      vision.getHubCenterFieldPositionFromTag(lockedTagId).ifPresent(center -> hubCenterField = center);
    } else {
      vision.getHubCenterFieldPositionFromLayout(Constants.HUB_TAG_IDS).ifPresent(center -> hubCenterField = center);
    }

    Pose2d robotPose = swerve.getPose();

    if (hubCenterField == null) {
      ChassisSpeeds noTargetSpeeds =
          swerve.getTargetSpeeds(
              xInput.getAsDouble(),
              yInput.getAsDouble(),
              robotPose.getRotation());
      swerve.driveFieldOriented(noTargetSpeeds);
      shooter.set(0.0);
      indexer.setduty(0.0);
      hopper.setduty(0.0);
      return;
    }

    ChassisSpeeds measuredFieldVelocity = swerve.getFieldVelocity();

    ChassisSpeeds commandedFieldVelocity =
        swerve.getTargetSpeeds(
            xInput.getAsDouble(),
            yInput.getAsDouble(),
            robotPose.getRotation());

    Translation2d measuredVel =
        new Translation2d(
            measuredFieldVelocity.vxMetersPerSecond,
            measuredFieldVelocity.vyMetersPerSecond);

    Translation2d commandedVel =
        new Translation2d(
            commandedFieldVelocity.vxMetersPerSecond,
            commandedFieldVelocity.vyMetersPerSecond);

    Translation2d estimatedVel =
        measuredVel.times(Constants.SHOT_MEASURED_VELOCITY_WEIGHT)
            .plus(commandedVel.times(1.0 - Constants.SHOT_MEASURED_VELOCITY_WEIGHT));

    Translation2d predictedReleasePose =
        robotPose.getTranslation().plus(estimatedVel.times(Constants.SHOT_POSE_PREDICTION_SECS));

    Translation2d toHub = hubCenterField.minus(predictedReleasePose);
    double distanceMeters = toHub.getNorm();

    Rotation2d desiredHeading =
        Rotation2d.fromRadians(Math.atan2(toHub.getY(), toHub.getX()));

    ChassisSpeeds driveSpeeds =
        swerve.getTargetSpeeds(
            xInput.getAsDouble(),
            yInput.getAsDouble(),
            desiredHeading);

    swerve.driveFieldOriented(driveSpeeds);

    double shooterRPM = shooter.rpmForDistanceMeters(distanceMeters);
    shooter.setMechanismVelocitySetpoint(RPM.of(shooterRPM));

    double headingErrorDeg =
        Math.abs(robotPose.getRotation().minus(desiredHeading).getDegrees());
    if (headingErrorDeg > 180.0) {
      headingErrorDeg = 360.0 - headingErrorDeg;
    }

    double headingToleranceDeg = currentlySeeingHubTag
        ? Constants.HUB_HEADING_TOLERANCE_DEG_WITH_VISION
        : Constants.HUB_HEADING_TOLERANCE_DEG_NO_VISION;

    boolean drivetrainReady = headingErrorDeg <= headingToleranceDeg;
    boolean shooterReady = shooter.getVelocity().in(RPM) >= shooterRPM * Constants.SHOOTER_READY_FRACTION;

    if (drivetrainReady && shooterReady) {
      indexer.setduty(-1.0);
      hopper.setduty(-1.0);
      CommandScheduler.getInstance().schedule(armOscillateCommand);
    } else {
      indexer.setduty(0.0);
      hopper.setduty(0.0);
    }

    SmartDashboard.putBoolean("HubAlign/SeeingHubTag", currentlySeeingHubTag);
    SmartDashboard.putNumber("HubAlign/LockedTagId", lockedTagId);
    SmartDashboard.putNumber("HubAlign/HubCenterX", hubCenterField.getX());
    SmartDashboard.putNumber("HubAlign/HubCenterY", hubCenterField.getY());
    SmartDashboard.putNumber("HubAlign/PredictedReleaseX", predictedReleasePose.getX());
    SmartDashboard.putNumber("HubAlign/PredictedReleaseY", predictedReleasePose.getY());
    SmartDashboard.putNumber("HubAlign/DistanceMeters", distanceMeters);
    SmartDashboard.putNumber("HubAlign/DesiredHeadingDeg", desiredHeading.getDegrees());
    SmartDashboard.putNumber("HubAlign/HeadingErrorDeg", headingErrorDeg);
    SmartDashboard.putNumber("HubAlign/ShooterTargetRPM", shooterRPM);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.set(0.0);
    indexer.setduty(0.0);
    hopper.setduty(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}