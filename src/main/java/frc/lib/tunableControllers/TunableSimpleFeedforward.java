package frc.lib.tunableControllers;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class TunableSimpleFeedforward extends TunableController{
    private SimpleMotorFeedforward ff;
    public TunableSimpleFeedforward(String name, double kS, double kV){
        super(name, kS, kV);
        ff = new SimpleMotorFeedforward(kS, kV);
    }
    public double calculate(double velocity){
        return ff.calculate(velocity);
    }
    @Override
    public void refresh(){
        readValues();
        ff = new SimpleMotorFeedforward(constants[0], constants[1]);
    }
}
