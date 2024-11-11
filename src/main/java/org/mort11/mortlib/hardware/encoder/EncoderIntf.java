package org.mort11.mortlib.hardware.encoder;

import edu.wpi.first.math.geometry.Rotation2d;

public interface EncoderIntf {

    public Rotation2d getPosition();

    public void setCanivore(String canivore);


}
