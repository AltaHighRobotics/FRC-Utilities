package utilities;

import utilities.PastaConstants;

public class CopyPastautonomous
{
	public CopyPastautonomous()
	{

	}

	public void setTankDriveAuto(double forward, double turn) 

		this.rightMotorFront.set(ControlMode.PercentOutput, forward * Constants.DRIVE_MAX_SPEED,
				DemandType.ArbitraryFeedForward, -turn * (Constants.DRIVE_MAX_SPEED * 0.666));
		this.leftMotorFront.set(ControlMode.PercentOutput, forward * Constants.DRIVE_MAX_SPEED,
				DemandType.ArbitraryFeedForward, turn * (Constants.DRIVE_MAX_SPEED * 0.666));
	}

	public void setSwerveDriveAuto(double forward, double turn)
	{

	}

	public void drivetrainPositionIntegration()
	{
	}

	public void driveForwardTo(final double waypointX, final double waypointY)
	{

	}

	public void driveBackwardsTo(final double waypointX, final double waypointY)
	{

	}

	public double[] getPos()
	{
		pos[0] = robotX;
		pos[1] = robotY;
		return pos;
	}

	/**
	 * DriveTrain drives to wayPoint specified
	 * 
	 * @param waypointX
	 * @param waypointY
	 * @param driveBackwards
	 * @return if it has reached a waypoint
	 * @deprecated
	 */
	public boolean setDriveToWaypoint(final double waypointX, final double waypointY, final boolean driveBackwards)
	{
		this.targetX = waypointX;
		this.targetY = waypointY;
		if (!driveBackwards) {
			this.targetHeading = Math.toDegrees(Math.atan2(this.targetY - this.robotY, this.targetX - this.robotX));
		} else {
			this.targetHeading = Math
					.toDegrees(Math.atan2(-(this.targetY - this.robotY), -(this.targetX - this.robotX)));
		}
		this.currentHeading = (double) this.navX.getYaw();
		this.headingRate = this.currentHeading - this.previousHeading;
		this.headingError = this.targetHeading - this.currentHeading;
		this.previousHeading = this.currentHeading;

		this.distanceError = Math
				.sqrt(Math.pow(this.targetY - this.robotY, 2) + Math.pow(this.targetX - this.robotX, 2));

		this.steeringPower = this.drivetrainHeadingPID.runVelocityPID(this.targetHeading, this.currentHeading,
				this.headingRate);

		if (Math.abs(this.headingError) < Constants.MAX_DRIVE_HEADING_ERROR) {
			this.drivePower = this.drivetrainSpeedPID.runPID(0, -this.distanceError);
			if (driveBackwards) {
				this.drivePower = -this.drivePower;
			}
		} else {
			this.drivePower = 0;
		}
		if (hasReachedWaypoint()) {
			stopMotors();
		} else {
			this.setArcadeDrive(this.drivePower, this.steeringPower);
		}
		return hasReachedWaypoint();
	}

	public boolean pointAtWaypoint(final double waypointX, final double waypointY)
	{

	}

	public void pointAtAngle(double targetAngle)
	{

	}

	public boolean hasReachedWaypoint()
	{
		return Math.abs(this.distanceError) < Constants.MAX_WAYPOINT_ERROR;
	}

	public void stopMotors()
	{
		this.leftMotorFront.neutralOutput();
		this.rightMotorFront.neutralOutput();
	}

	public void setPos(double x, double y)
	{
		this.robotX = x;
		this.robotY = y;
	}

	public void resetYaw()
	{
		this.navX.zeroYaw();
	}
}
