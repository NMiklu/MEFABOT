import numpy as np
class normalize_joints(object):
    
    """
    Internal use to help with joint limits, multiple revolutions,
    and last joint angles
    """
    
    def __init__(self, robot, last_joints, _ignore_limits = False):
        self._lower_limit = robot.joint_lower_limit
        self._upper_limit = robot.joint_upper_limit
        self._check_limits = self._lower_limit is not None \
            and self._upper_limit is not None and not _ignore_limits
                  
        self._last_joints = last_joints
        self._use_last_joints = last_joints is not None
            
    def normalize(self, joint, theta):
        if self._check_limits:
            l = self._lower_limit[joint]
            u = self._upper_limit[joint]
            
            if not (l < theta and theta < u ):
                a = 2*np.pi*np.array([-1,1])
                b = a + theta
                c = np.argwhere(np.logical_and(l < b,  b < u))
                if len(c) == 0:
                    return None                
                theta += (a[c[0]]).item()
        
        if self._use_last_joints:
            diff = self._last_joints[joint] - theta                               
            n_diff = np.floor_divide(diff, 2*np.pi)
            r_diff = np.remainder(diff, 2*np.pi)
            if (r_diff > np.pi): 
                n_diff+=1
            if np.abs(n_diff) > 0:
                if not self._check_limits:
                    theta += 2*np.pi*n_diff
                else:                
                    theta_v = theta + 2*np.pi*np.arange(n_diff, -np.sign(n_diff), - np.sign(n_diff))
                    theta_ind = np.argwhere(np.logical_and(l < theta_v, theta_v < u))
                    theta = theta_v[theta_ind[0]].item()
                
        return theta
                                                        
                
    
    def __call__(self, joint, theta):
                        
        theta_normed=[]
        if len(np.shape(joint)) == 0:
            for t1 in theta:
                t3=self.normalize(joint, t1)
                if t3 is not None:
                    theta_normed.append(t3)
        else:
            for t1 in theta:
                t3 = tuple([self.normalize(j2,t2) for j2, t2 in zip(joint, t1)])
                if not None in t3:
                    theta_normed.append(t3)                 
        
        if (not self._use_last_joints) or len(theta_normed) < 2:
            return theta_normed
        
        theta_last = np.array(np.take(self._last_joints, joint))
        if len(theta_last.shape) == 0:
            theta_dist = np.abs(np.subtract(theta_normed,theta_last))            
        else:
            theta_dist = np.linalg.norm(np.subtract(theta_normed,theta_last), axis=1)
        
        #Heuristic pruning of last_joints
        theta_ret1 = [t for t in theta_normed if np.all(np.less(np.abs(t - theta_last), np.pi/2.0))]        
        if len(theta_ret1) == 1:
            return theta_ret1
        if len(theta_ret1) == 0:
            theta_ret1 = theta_normed
        
        return [theta_normed[i] for i in list(np.argsort(theta_dist))]