import numpy as np

def single_meas_func(C1,C0,k,b,dist):
    return k*(dist-C1)**b+C0

def joint_meas_func(C1s,C0s,ks,bs,x,ps):

    # Casting for the compatibility of jax.numpy

    C1=np.array(C1s)
    C0=np.array(C0s)
    k=np.array(ks)
    b=np.array(bs)
    p=np.array(ps)

    # Keep in mind that x is a vector of [q,q'], thus only the first half of components are observable.    
    dists=np.linalg.norm(x[:len(x)//2]-p,axis=1)

    return single_meas_func(C1,C0,k,b,dists) 

def dhdr(r,C1s,C0s,ks,bs):
    return ks*bs*(r-C1s)**(bs-1)
def d2hdr2(r,C1s,C0s,ks,bs):
    return dhdr(r,C1s,C0s,ks,bs)*(bs-1)/(r-C1s)

def analytic_dhdz(x,ps,C1s,C0s,ks,bs):

    q = x.flatten()
    q = q[:len(q)//2]
    dhdq = analytic_dhdq(q,ps,C1s,C0s,ks,bs)
    # return jnp.hstack([dhdq,np.zeros(dhdq.shape)])
    return np.hstack([dhdq,np.zeros(dhdq.shape)])

def analytic_dhdq(q,ps,C1s,C0s,ks,bs):
    # rs = jnp.linalg.norm(ps-q,axis=1)
    rs = np.linalg.norm(ps-q,axis=1)
   
    r_hat = ((ps-q).T/rs).T
    d = dhdr(rs,C1s,C0s,ks,bs)
    dhdq=-(d * r_hat.T).T
    return dhdq

def analytic_FIM(q,ps,C1s,C0s,ks,bs):
    # rs = np.linalg.norm(ps-q,axis=1)
    rs = np.linalg.norm(ps-q,axis=1)
    r_hat = ((ps-q).T/rs).T


    d = dhdr(rs,C1s,C0s,ks,bs)
    dd = d2hdr2(rs,C1s,C0s,ks,bs)       

    As = (-d*r_hat.T).T
   

    return As.T.dot(As) # Current FIM

def F_single(dh,qhat,ps):
    A = dh(qhat,ps)
    return A.T.dot(A)

def joint_F_single(qhat,ps,C1,C0,k,b): # Verified to be correct.
    # The vectorized version of F_single.
    # The output shape is (N_sensor, q_dim, q_dim).
    # Where output[i]=F_single(dh,qhat,ps[i])
    A = analytic_dhdq(qhat,ps,C1s=C1,C0s=C0,ks=k,bs=b)
    return A[:,np.newaxis,:]*A[:,:,np.newaxis]


def analytic_dLdp(q,ps,C1s,C0s,ks,bs,FIM=None):
    """
        The gradient is taken with respect to all the ps passed in. 

        The FIM is by default calculated internally, but if it is passed in, will
        use the passed in FIM for the calculation of Q below.
    """
  
    rs = np.linalg.norm(ps-q,axis=1)
    r_hat = ((ps-q).T/rs).T
    t_hat=np.zeros(r_hat.shape)
    t_hat[:,0]=-r_hat[:,1]
    t_hat[:,1]=r_hat[:,0]

    d = dhdr(rs,C1s,C0s,ks,bs)
    dd = d2hdr2(rs,C1s,C0s,ks,bs)

    wrhat=(d*r_hat.T).T

    if FIM is None:
        Q = np.linalg.inv(wrhat.T.dot(wrhat)) # Default calculation of FIM^-1
    else:
        # print('Coordinating')
        if np.linalg.matrix_rank(FIM) < 2:
            FIM = FIM + 1e-9*np.eye(2)
        Q = np.linalg.inv(FIM) # Using the passed in FIM.

    c1 = -2*d*dd*np.linalg.norm(Q.dot(r_hat.T),axis=0)**2
    c2 = -2*(1/rs)*(d**2)*np.einsum('ij,ij->j',Q.dot(r_hat.T),Q.dot(t_hat.T))

    return (c1*r_hat.T+c2*t_hat.T).T

def local_dLdp(q,p,p_neighborhood,C1s,C0s,ks,bs):
    
    local_FIM = analytic_FIM(q,p_neighborhood,C1s,C0s,ks,bs)

    return analytic_dLdp(q,p,C1s,C0s,ks,bs,FIM = local_FIM)

