package com.team1678.frc2021.auto;

import com.team1678.frc2021.auto.modes.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING, 
        TEST_PATH_AUTO,
        // RIGHT_TEN_BALL_AUTO,
        // RIGHT_FIVE_NEAR_BALL_AUTO,
        // RIGHT_FIVE_FAR_BALL_AUTO,
        // LEFT_EIGHT_NEAR_BALL_AUTO,
        // LEFT_EIGHT_FAR_BALL_AUTO,
        // LEFT_SIX_NEAR_BALL_AUTO,
        // LEFT_SIX_FAR_BALL_AUTO,
        // SHOT_CENTER_BACK_AUTO,
        SHOT_CENTER_FORWARD_AUTO,
        // SHOT_LEFT_BACK_AUTO,
        // SHOT_LEFT_FRONT_AUTO,
        // AIM_TEST_AUTO
    }

    private DesiredMode mCachedDesiredMode = null;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test Path", DesiredMode.TEST_PATH_AUTO);
        mModeChooser.addOption("Shot Center Forward", DesiredMode.SHOT_CENTER_FORWARD_AUTO);
        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if (mCachedDesiredMode != desiredMode) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
        case DO_NOTHING:
            return Optional.of(new DoNothingMode());
        
        case TEST_PATH_AUTO:
            return Optional.of(new TestStraightPathMode());
        
        // case RIGHT_TEN_BALL_AUTO:
        //     return Optional.of(new RightTenMode());

        // case RIGHT_FIVE_NEAR_BALL_AUTO:
        //     return Optional.of(new RightFiveNearMode());

        // case RIGHT_FIVE_FAR_BALL_AUTO:
        //     return Optional.of(new RightFiveFarMode());

        // case LEFT_EIGHT_NEAR_BALL_AUTO:
        //     return Optional.of(new LeftEightNearMode());

        // case LEFT_EIGHT_FAR_BALL_AUTO:
        //     return Optional.of(new LeftEightFarMode());
            
        // case LEFT_SIX_FAR_BALL_AUTO:
        //     return Optional.of(new LeftSixFarMode());

        // case LEFT_SIX_NEAR_BALL_AUTO:
        //     return Optional.of(new LeftSixNearMode());

        // case SHOT_CENTER_BACK_AUTO:
        //     return Optional.of(new ShotCenterBackMode());

        case SHOT_CENTER_FORWARD_AUTO:
            return Optional.of(new ShotCenterForwardMode());
        
        // case SHOT_LEFT_BACK_AUTO:
        //     return Optional.of(new ShotLeftBackMode());

        // case SHOT_LEFT_FRONT_AUTO:
        //     return Optional.of(new ShotLeftFrontMode());

        // case AIM_TEST_AUTO:
        //     return Optional.of(new AimTestAuto());
            
        default:
            System.out.println("ERROR: unexpected auto mode: " + mode);
            break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        if (!mAutoMode.isPresent()) {
            return Optional.empty();
        }
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DO_NOTHING;
    }
}
