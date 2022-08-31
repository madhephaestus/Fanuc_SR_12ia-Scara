import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.common.DeviceManager

MobileBase base=DeviceManager.getSpecificDevice( "Fanuc_Delta_DR",{
	return ScriptingEngine.gitScriptRun(	"https://github.com/madhephaestus/Fanuc_Delta_DR-3iB.git",
									"Fanuc_Delta_DR-3iB.xml",
									null
	  )
})

double baseCircleDiam = 280
double powerLinkLen = 565
double passiveLinkLen = 1260
double eoaPlatRad =75
double passiveSepDist = 166.8

for(DHParameterKinematics k:base.getAllDHChains()) {
	String name = k.getScriptingName()
	double yOffset = passiveSepDist/2
	double rot =0
	if(name.endsWith("a"))
		yOffset*=-1;
	if(name.contains("2")) {
		rot=120
	}
	if(name.contains("3")) {
		rot=240
	}
	
}