
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
DHParameterKinematics tar=null
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
	def rotation = new TransformNR(0,0,0,new RotationNR(0, rot, 0))
	TransformNR limbRoot = rotation
		.times(new TransformNR(baseCircleDiam/2,yOffset,0,new RotationNR(0, 0,0)))
	limbRoot.setRotation(new RotationNR(0,rot-90,-89.9999))
	
	k.setDH_D(1, powerLinkLen)
	k.setDH_D(3, passiveLinkLen)
	ParallelGroup baseGetParallelGroup = base.getParallelGroup(k)
	if(k.getNumberOfLinks()==8) {
		k.setDH_R(5, passiveSepDist/2)
		k.setDH_R(6, eoaPlatRad)
		tar=k
	}
	if(baseGetParallelGroup!=null && !name.endsWith("1")) {
		double centerx = eoaPlatRad
		double centery =passiveSepDist/2
		TransformNR	tipLoc = rotation
							.times(new TransformNR(centerx,centery,0,new RotationNR(0, 0,0)))
		tipLoc.setRotation(new RotationNR())
				.times(new TransformNR(-centerx,-centery,0,new RotationNR()))
		println name
		println tipLoc
		baseGetParallelGroup.setTipOffset(k, tipLoc)
	}

	println limbRoot
	k.setRobotToFiducialTransform(limbRoot)
}
base.getParallelGroup(tar).setDesiredTaskSpaceTransform(new TransformNR(), 0)

