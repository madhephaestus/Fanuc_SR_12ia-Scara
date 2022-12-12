import java.text.DecimalFormat

import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager

DecimalFormat df = new DecimalFormat("000.00");
MobileBase base=DeviceManager.getSpecificDevice( "Fanuc_Scara_SR_12ia",{ScriptingEngine.gitScriptRun(	"https://github.com/madhephaestus/Fanuc_SR_12ia-Scara.git", "Fanuc_SR_12ia.xml", null )})


DHParameterKinematics mainLimb = base.getAllDHChains().get(0)
double[] links =[143.77	, 045.61	, -96.00	, 153.90	, 000.00	]

//mainLimb.setDesiredJointSpaceVector(links,0)

TransformNR Tip= mainLimb.forwardOffset(mainLimb.forwardKinematics(links))
println "Home: "+Tip

double[] linksComuted = mainLimb.inverseKinematics(mainLimb.inverseOffset(Tip))
//mainLimb.setDesiredJointSpaceVector(links,0)
//mainLimb.setDesiredTaskSpaceTransform(Tip, 0)

println links.collect{df.format(it)+"\t"}
println "Computed:"
println linksComuted.collect{df.format(it)+"\t"}



def difference =[]
for(int i=0;i<linksComuted.length;i++) {
	difference.add(df.format(links[i]-linksComuted[i])+"\t")
}
println "Difference:"
println difference

def lim =[]
for(int i=0;i<linksComuted.length;i++) {
	lim.add(df.format(Math.toDegrees(mainLimb.getDH_Theta(i)))+"\t")
}
println "\n\nThetas "+lim

return null