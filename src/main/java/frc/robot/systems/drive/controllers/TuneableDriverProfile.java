// REBELLION 10014

package frc.robot.systems.drive.controllers;

import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;

public class TuneableDriverProfile {
    private LoggedTunableNumber mLinearScalar;
    private LoggedTunableNumber mExtraXYStickDeadband;
    private LoggedTunableNumber mLinearInputsExponent;
    private LoggedTunableNumber mRotationScalar;
    private LoggedTunableNumber mRotationInputsExponent;
    private LoggedTunableNumber mExtraRotationDeadband;
    private LoggedTunableNumber mSniperControl;

    public TuneableDriverProfile(
            String pKey,
            double pDefaultLinearScalar,
            double pDefaultExtraXYStickDeadband,
            double pDefaultLinearInputsExponent,
            double pDefaultRotationScalar,
            double pDefaultRotationInputExponent,
            double pDefaultExtraRotationDeadband,
            double pDefaultSniperControl) {
        mLinearScalar = new LoggedTunableNumber("Drive/Teleop/" + pKey + "/LinearScalar", pDefaultLinearScalar);
        mExtraXYStickDeadband = new LoggedTunableNumber("Drive/Teleop/" + pKey + "/ExtraXYStickDeadband", pDefaultExtraXYStickDeadband);
        mLinearInputsExponent =
            new LoggedTunableNumber("Drive/Teleop/" + pKey + "/LinearInputsExponent", pDefaultLinearInputsExponent);
        mRotationScalar = new LoggedTunableNumber("Drive/Teleop/" + pKey + "/RotationScalar", pDefaultRotationScalar);
        mRotationInputsExponent = new LoggedTunableNumber(
            "Drive/Teleop/" + pKey + "/RotationInputsExponent", pDefaultRotationInputExponent);
        mExtraRotationDeadband =
            new LoggedTunableNumber("Drive/Teleop/" + pKey + "/ExtraRotationStickDeadband", pDefaultExtraRotationDeadband);
        mSniperControl = new LoggedTunableNumber("Drive/Teleop/" + pKey + "/SniperControl", pDefaultSniperControl);
    }

    public TuneableDriverProfile(DriverProfiles pDefaults) {
        this(
            pDefaults.key(),
            pDefaults.linearScalar(),
            pDefaults.extraXYStickDeadband(),
            pDefaults.linearExponent(),
            pDefaults.rotationalScalar(),
            pDefaults.rotationalScalar(),
            pDefaults.extraRotationDeadband(),
            pDefaults.sniperScalar()
        );
    }

    public LoggedTunableNumber linearScalar() {
        return mLinearScalar;
    }

    public LoggedTunableNumber extraXYStickDeadband() {
        return mExtraXYStickDeadband;
    }

    public LoggedTunableNumber linearInputsExponent() {
        return mLinearInputsExponent;
    }

    public LoggedTunableNumber rotationScalar() {
        return mRotationScalar;
    }

    public LoggedTunableNumber rotationInputsExponent() {
        return mRotationInputsExponent;
    }

    public LoggedTunableNumber extraRotationDeadband() {
        return mExtraRotationDeadband;
    }

    public LoggedTunableNumber sniperControl() {
        return mSniperControl;
    }
}
