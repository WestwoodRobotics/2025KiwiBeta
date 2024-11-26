package CustomLibs.QualityOfLife;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.utils.SparkModels;


import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

public class NeoSpark extends SparkBase {
    SparkBaseConfig current_config;
    SparkBase sparkBase;
    SparkModel s;


    public NeoSpark(SparkMax sparkMax) {
        super(sparkMax.getDeviceId(), 
              MotorType.kBrushless, 
              SparkModel.SparkMax);
        this.sparkBase = sparkMax;
        current_config = new SparkMaxConfig();
    }

    public NeoSpark(SparkFlex sparkFlex) {
        super(sparkFlex.getDeviceId(), 
              MotorType.kBrushless, 
              SparkModel.SparkFlex);
        this.sparkBase = sparkFlex;
        current_config = new SparkFlexConfig();
    }

    public NeoSpark(int port, MotorType type, SparkModels model) {
        super(port, type, ((model == SparkModels.SparkMax) ? SparkModel.SparkMax : SparkModel.SparkFlex));
        if (s == SparkModel.SparkMax) {
            current_config = new SparkMaxConfig();
        } else {
            current_config = new SparkFlexConfig();
        }
    }


    public void configureNeoSparkBase(IdleMode idleMode, ResetMode resetMode, PersistMode persistMode) {
        current_config.idleMode(idleMode);
        this.configure(current_config, resetMode, persistMode);
    }
}