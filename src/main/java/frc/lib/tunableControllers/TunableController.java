package frc.lib.tunableControllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableController {
    private String name;
    protected double[] constants;
    protected TunableController(String name, double... constants){
        this.name = name;
        double[] defaultData = constants;
        
        double[] data = SmartDashboard.getNumberArray(name, defaultData);
         
        this.constants = data;

        SmartDashboard.putNumberArray(name, data);
    }
    public void refresh(){

    } 
    protected void readValues(){
        double[] defaultData = constants;
        double[] data = SmartDashboard.getNumberArray(name, defaultData);
        constants = data;
    }  
}



