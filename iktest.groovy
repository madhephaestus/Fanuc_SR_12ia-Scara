import java.text.DecimalFormat

import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager

DecimalFormat df = new DecimalFormat("000.00");
MobileBase base=DeviceManager.getSpecificDevice( "Fanuc_Delta_DR",{ScriptingEngine.gitScriptRun(	"https://github.com/madhephaestus/Fanuc_Delta_DR-3iB.git", "Fanuc_Delta_DR-3iB.xml", null )})


DHParameterKinematics mainLimb = base.getAllDHChains().get(0)
double[] links = [-25,-35,30,0,4,0,0]

mainLimb.setDesiredJointSpaceVector(links,0)

TransformNR Tip= mainLimb.forwardOffset(mainLimb.forwardKinematics(links))

double[] linksComuted = mainLimb.inverseKinematics(mainLimb.inverseOffset(Tip))

println links.collect{df.format(it)+"\t"}

println linksComuted.collect{df.format(it)+"\t"}



def difference =[]
for(int i=0;i<links.length;i++) {
	difference.add(df.format(links[i]-linksComuted[i])+"\t")
}
println difference

def lim =[]
for(int i=0;i<links.length;i++) {
	lim.add(df.format(Math.toDegrees(mainLimb.getDH_Theta(i)))+"\t")
}
println "\n\nThetas "+lim

return null