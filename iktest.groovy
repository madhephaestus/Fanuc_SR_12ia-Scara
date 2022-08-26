import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager

//Your code here

MobileBase base=DeviceManager.getSpecificDevice( "Fanuc_Delta_DR",{ScriptingEngine.gitScriptRun(	"https://github.com/madhephaestus/Fanuc_Delta_DR-3iB.git", "Fanuc_Delta_DR-3iB.xml", null )})



DHParameterKinematics mainLimb = base.getAllDHChains().get(0)
double[] links = [-45,-25,30,0,0,0,0]

mainLimb.setDesiredJointSpaceVector(links,0)

TransformNR Tip= mainLimb.forwardOffset(mainLimb.forwardKinematics(links))

double[] linksComuted = mainLimb.inverseKinematics(mainLimb.inverseOffset(Tip))

println links

println linksComuted

def difference =[]
for(int i=0;i<links.length;i++) {
	difference.add(links[i]-linksComuted[i])
}
println difference

return null