package frc.lib.tunableControllers;

import edu.wpi.first.math.controller.ArmFeedforward;

public class TunableArmFeedforward extends TunableController{
    private ArmFeedforward ff;
    public TunableArmFeedforward(String name, double kS, double kG, double kV){
        super(name, kS, kG, kV);
        ff = new ArmFeedforward(kS, kG, kV);
    }
    public double calculate(double positionRadians, double velocity){
        return ff.calculate(positionRadians, velocity);
    }
    @Override
    public void refresh(){
        readValues();
        ff = new ArmFeedforward(constants[0], constants[1], constants[2]);
    }
}
