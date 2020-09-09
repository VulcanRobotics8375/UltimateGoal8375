package org.vulcanrobotics.robotcorelib.motion;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.vulcanrobotics.robotcorelib.dashboard.hardware.Odometer;
import org.vulcanrobotics.robotcorelib.framework.RobotCoreLibException;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;

public abstract class MotionProfile {

     List<Odometer> odometers = new ArrayList<>();
    int odometerNum;
    Properties properties = new Properties();

    public MotionProfile(int odometerNum, Odometer... odometer) throws RobotCoreLibException {
        this.odometerNum = odometerNum;
        if(odometer.length > odometerNum) {
            throw new RobotCoreLibException();
        }
        odometers.addAll(Arrays.asList(odometer));
    }

    public  void init() throws RobotCoreLibException {
        try {
            String path = AppUtil.getInstance().getSettingsFile("robotconfig.properties").getAbsolutePath();
            
            //set up file streams for reading and writing to the properties file
            FileInputStream input = new FileInputStream(path);
            properties.load(input);
        }
        catch(IOException e) {
            e.printStackTrace();
        }
    }

    public abstract void update();

    public abstract void calibrate(double gain);

    protected Odometer getOdometerByName(String name) {
        for(Odometer o : odometers) {
            if(o.getName().equals(name))
                return o;
        }
        return null;
    }

}
