package frc.lib.tunableControllers;

import edu.wpi.first.math.controller.ArmFeedforward;

public class TunableArmFeedforward extends TunableController{
    private ArmFeedforward ff;
    private static final String[] knames = {"kS", "kG", "kV"};

    public TunableArmFeedforward(String name, double kS, double kG, double kV){
        super(name, knames, kS, kG, kV);
        ff = new ArmFeedforward(kS, kG, kV);
    }
    public double calculate(double positionRadians, double velocity){
        return ff.calculate(positionRadians, velocity);
    }
    @Override
    public void refresh(){
        if(parameterUpdate()){
            ff = new ArmFeedforward(constants[0], constants[1], constants[2]);
        }
    }
}
