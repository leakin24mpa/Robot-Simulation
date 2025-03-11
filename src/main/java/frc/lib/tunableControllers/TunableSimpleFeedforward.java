package frc.lib.tunableControllers;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class TunableSimpleFeedforward extends TunableController{
    private SimpleMotorFeedforward ff;
    private static final String[] knames = {"kS", "kV"};
    public TunableSimpleFeedforward(String name, double kS, double kV){
        super(name, knames, kS, kV);
        ff = new SimpleMotorFeedforward(kS, kV);
    }
    public double calculate(double velocity){
        return ff.calculate(velocity);
    }
    @Override
    public void refresh(){
        if(parameterUpdate()){
            ff = new SimpleMotorFeedforward(constants[0], constants[1]);
        }
    }
}
