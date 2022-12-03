package frc.robot.systems.compressor;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompressorSubsystem extends SubsystemBase {
    private final Compressor compressor;

    public CompressorSubsystem(PneumaticConfig cfg) {
        compressor = !cfg.simulated ? new Compressor(cfg.compressorID, cfg.pcmType) : null;
        addChild("compressor", compressor);
    }

    /**
     * Start holding ~120 PSI.
     */
    public void start() {
        compressor.enableDigital();
    }

    /**
     * Stop actively regulating pressure
     */
    public void stop() {
        compressor.disable();
    }
}
