
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.addons.kinematics.parallel.ParallelGroup
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
	TransformNR limbRoot = new TransformNR(0,0,0,new RotationNR(0, rot, 0))
		.times(new TransformNR(baseCircleDiam,yOffset,0,new RotationNR(0, 0,0)))
	limbRoot.setRotation(new RotationNR(0,rot-90,-89.9999))
	
	k.setDH_D(1, powerLinkLen)
	k.setDH_D(3, passiveLinkLen)
	ParallelGroup baseGetParallelGroup = base.getParallelGroup(k)
	if(baseGetParallelGroup!=null && !name.endsWith("1")) {
		double centerx = passiveSepDist/2
		double centery =eoaPlatRad
		TransformNR local = new TransformNR(-centerx -(centerx * Math.sin(Math.toRadians(rot))),
		-centery,
		0,new RotationNR() )
		baseGetParallelGroup.setTipOffset(k, local)
	}
	
	k.setRobotToFiducialTransform(limbRoot)
}