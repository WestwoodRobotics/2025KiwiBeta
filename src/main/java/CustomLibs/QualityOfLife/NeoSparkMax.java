package CustomLibs.QualityOfLife;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class NeoSparkMax extends SparkMax {
    SparkMaxConfig current_config;
    
    public NeoSparkMax(int port, MotorType type) {
        super(port, type);
        current_config = new SparkMaxConfig();
    }

    public void configureNeoSparkMax(IdleMode idleMode, ResetMode resetMode, PersistMode persistMode) {
        
        current_config.idleMode(idleMode);
        this.configure(current_config, resetMode, persistMode);
    }

    public void setNeoSparkMaxConfig(SparkMaxConfig config){
        this.current_config = config;
        this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SparkMaxConfig getNeoSparkMaxConfig(){
        return this.current_config;
    }
    
}
