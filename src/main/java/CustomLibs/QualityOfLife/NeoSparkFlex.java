package CustomLibs.QualityOfLife;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;




public class NeoSparkFlex extends SparkFlex{

    SparkFlexConfig current_config;
    
    public NeoSparkFlex(int port, MotorType type) {
        super(port, type);
        current_config = new SparkFlexConfig();
    }

    public void configureNeoSparkFlex(IdleMode idleMode, ResetMode resetMode, PersistMode persistMode) {
        
        current_config.idleMode(idleMode);
        this.configure(current_config, resetMode, persistMode);
    }

    public void setNeoSparkFlexConfig(SparkFlexConfig config){
        this.current_config = config;
        this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SparkFlexConfig getNeoSparkFlexConfig(){
        return this.current_config;
    }



}
