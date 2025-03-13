import torch
import pypose as pp
# r = pp.randn_so3(2, requires_grad=True)

# create a random so(3) element
r= pp.so3(torch.tensor([0.0,0.0,1.0],requires_grad=True))

# exponential map to quaternion
R = r.Exp()
print(R)

# create a vector
p0=torch.tensor([0.0,1.0,0.0])
print(p0)

# rotate the vector
p1 = R@p0
print(p1)

# calculate the gradient of the rotated vector with respect to the so(3) element
#  


