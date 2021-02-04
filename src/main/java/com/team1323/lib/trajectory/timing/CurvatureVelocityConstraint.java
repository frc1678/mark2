package com.team1323.lib.trajectory.timing;

import com.team1323.lib.geometry.UnwrappablePose2dWithCurvature;
import com.team1678.frc2021.Constants;

public class CurvatureVelocityConstraint implements TimingConstraint<UnwrappablePose2dWithCurvature>{

	@Override
	public double getMaxVelocity(final UnwrappablePose2dWithCurvature state){
		return Constants.kSwerveMaxSpeedInchesPerSecond / (1 + Math.abs(4.0*state.getCurvature()));//6.0
	}
	
	@Override
	public MinMaxAcceleration getMinMaxAcceleration(final UnwrappablePose2dWithCurvature state, final double velocity){
		return MinMaxAcceleration.kNoLimits;
	}
	
}
