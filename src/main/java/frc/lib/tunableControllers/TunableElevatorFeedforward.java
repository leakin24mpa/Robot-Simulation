package frc.lib.tunableControllers;

import edu.wpi.first.math.controller.ElevatorFeedforward;

public class TunableElevatorFeedforward extends TunableController{
    private ElevatorFeedforward ff;
    public TunableElevatorFeedforward(String name, double kS, double kG, double kV){
        super(name, kS, kG, kV);
        ff = new ElevatorFeedforward(kS, kG, kV);
    }
    public double calculate(double velocity){
        return ff.calculate(velocity);
    }
    @Override
    public void refresh(){
        readValues();
        ff = new ElevatorFeedforward(constants[0], constants[1], constants[2]);
    }
}