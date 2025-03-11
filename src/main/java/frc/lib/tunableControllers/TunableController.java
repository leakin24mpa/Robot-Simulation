package frc.lib.tunableControllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableController {
    private String name;
    protected double[] constants;
    private String[] constantNames;
    protected TunableController(String name, String[] names, double... constants){
        this.name = name;
        this.constantNames = names;
        
        this.constants = constants;
        for(int i = 0; i < this.constants.length; i++){
            SmartDashboard.putNumber(name + "/" + names[i], this.constants[i]);
        }
        
    }
    protected boolean parameterUpdate(){
        boolean hasUpdate = false;
        for(int i = 0; i < this.constants.length; i++){
            double newValue = SmartDashboard.getNumber(name + "/" + constantNames[i], this.constants[i]);
            if(newValue != this.constants[i]){
                this.constants[i] = newValue;
                hasUpdate = true;
            }
        }   
        return hasUpdate;
    }
    public void refresh(){

    } 
}



