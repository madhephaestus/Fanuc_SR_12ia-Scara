import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.AbstractLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.common.DeviceManager
MobileBase base=DeviceManager.getSpecificDevice( "Fanuc_Delta_DR",{
	return ScriptingEngine.gitScriptRun(	"https://github.com/madhephaestus/Fanuc_Delta_DR-3iB.git",
									"Fanuc_Delta_DR-3iB.xml",
									null
	  )
})

for(DHParameterKinematics kin:base.getAllDHChains()) {
	println "Setting values for "+kin.getScriptingName()
	for(int i=1;i<kin.getNumberOfLinks();i++) {
		lf=kin.getLinkConfiguration(i);
		lf.setDeviceTheoreticalMax(360)
		lf.setDeviceTheoreticalMin(-360)
		lf.setUpperLimit(360);
		lf.setLowerLimit(-360)

	}
}