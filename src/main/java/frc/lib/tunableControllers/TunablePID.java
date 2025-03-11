package frc.lib.tunableControllers;

import edu.wpi.first.math.controller.PIDController;

public class TunablePID extends TunableController{
    private PIDController pid;
    private static final String[] knames = {"kP", "kI", "kD"};
    public TunablePID(String name, double kP, double kI, double kD){
        super(name, knames, kP, kI, kD);
        pid = new PIDController(kP, kI, kD);
    }
    public double calculate(double measurement, double setpoint){
        return pid.calculate(measurement,setpoint);
    }
    @Override
    public void refresh(){
        if(parameterUpdate()){
            pid.setPID(constants[0], constants[1], constants[2]);
        }
    }
}
