import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.common.DeviceManager

//Your code here

MobileBase base=DeviceManager.getSpecificDevice( "Fanuc_Delta_DR",{ScriptingEngine.gitScriptRun(	"https://github.com/madhephaestus/Fanuc_Delta_DR-3iB.git", "Fanuc_Delta_DR-3iB.xml", null )})



DHParameterKinematics mainLimb = base.getAllDHChains().get(0)
double[] links = [-45,-45,15,0,0,0,0]

mainLimb.setDesiredJointSpaceVector(links,0)

mainLimb.setDesiredTaskSpaceTransform(mainLimb.forwardOffset(mainLimb.forwardKinematics(links)), 0)


return null