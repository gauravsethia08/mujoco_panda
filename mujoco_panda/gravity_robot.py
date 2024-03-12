# import mujoco_py as mjp
import mujoco as mj

class GravityRobot(object):
    """
    Robot instance for gravity compensation only.
    """

    def __init__(self, model_path):
        self.model = mj.MjModel.from_xml_path(model_path)
        self.data = mj.MjData(self.model)

        self._controllable_joints = self.get_controllable_joints()

        # self._all_joints = [self.model.joint_name2id(
        #     j) for j in self.model.joint_names]
        self._all_joints = [
            self.model.joint(j).id for j in self.model.jnt_qposadr]

    def get_controllable_joints(self):
        """
        Return list of movable (actuated) joints in the given model

        :return: list of indices of controllable joints
        :rtype: [int] (size: self._nu)
        """
        trntype = self.model.actuator_trntype  # transmission type (0 == joint)
        # transmission id (get joint actuated)
        trnid = self.model.actuator_trnid

        mvbl_jnts = []
        for i in range(trnid.shape[0]):
            if trntype[i] == 0 and trnid[i, 0] not in mvbl_jnts:
                mvbl_jnts.append(trnid[i, 0])

        return sorted(mvbl_jnts)
