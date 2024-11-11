package org.mort11.mortlib.hardware.encoder;

import org.mort11.mortlib.hardware.brands.ctre.CANCoderEncoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class Encoder implements EncoderIntf {

    public EncoderTypeEnum encoderType;
    public int ID;

    public EncoderIntf encoder;
    
    public Encoder(EncoderTypeEnum encoderType, int ID) {
        this.encoderType = encoderType;
        this.ID = ID;

        switch (encoderType) {
            case CANCODER:
                encoder = new CANCoderEncoder(ID);
                break;
        }
    }

    public void setCanivore(String canivore) {
        encoder.setCanivore(canivore);
    }

    public Rotation2d getPosition() {
        return encoder.getPosition();
    }

    public EncoderTypeEnum getEncoderType() {
        return encoderType;
    }

}