import math

# Vector3 class 
class V3(object):
	def __init__(self, v, *argv):
		if len(argv) == 2:
			self.x = float(v)
			self.y = float(argv[0])
			self.z = float(argv[1])
		elif len(v) == 3:
			self.x = float(v[0])
			self.y = float(v[1])
			self.z = float(v[2])
		else:
			raise ValueError('V3 error!')
	
	def __len__(self):
		return 3

	def __getitem__(self, b):
		if b == 0:
			return self.x
		elif b == 1:
			return self.y
		elif b == 2:
			return self.z
		else:
			raise ValueError('V3 error!')
			
	def __setitem__(self, b, c):
		if type(c) == float or type(c) == int:
			if b == 0:
				self.x = c;
			elif b == 1:
				self.y = c;
			elif b == 2:
				self.z = c;
			else:
				raise ValueError('V3 error!')
		else:
			raise ValueError('V3 error!')
			
	def __add__(self, b):
		if type(b) == float or type(b) == int:
			return V3(self.x + b, self.y + b, self.z + b)
		elif len(b) == 3:
			return V3(self.x + b[0], self.y + b[1], self.z + b[2])
		else:
			raise ValueError('V3 error!')

	def __radd__(self, b):
		if type(b) == float or type(b) == int:
			return V3(self.x + b, self.y + b, self.z + b)
		elif len(b) == 3:
			return V3(self.x + b[0], self.y + b[1], self.z + b[2])
		else:
			raise ValueError('V3 error!')

	def __sub__(self, b):
		if type(b) == float or type(b) == int:
			return V3(self.x - b, self.y - b, self.z - b)
		elif len(b) == 3:
			return V3(self.x - b[0], self.y - b[1], self.z - b[2])
		else:
			raise ValueError('V3 error!')

	def __rsub__(self, b):
		if len(b) == 3:
			return V3(b[0] - self.x, b[1] - self.y, b[2] - self.z) # Because '-' is anticommutative.
		else:
			raise ValueError('V3 error!')

	def __mul__(self, b):
		if type(b) == float or type(b) == int:
			return V3(self.x * b, self.y * b, self.z * b)
		elif len(b) == 3:
			return V3(self.x * b[0], self.y * b[1], self.z * b[2]) # This is matlab .*
		else:
			raise ValueError('V3 error!')

	def __rmul__(self, b):
		if type(b) == float or type(b) == int:
			return V3(self.x * b, self.y * b, self.z * b)
		elif len(b) == 3:
			return V3(self.x * b[0], self.y * b[1], self.z * b[2]) # This is matlab ./
		else:
			raise ValueError('V3 error!')

	def __pow__(self, b):
		if type(b) == float or type(b) == int:
			return V3(self.x**b, self.y**b, self.z**b) # This is matlab .^
		elif len(b) == 3:
			return V3(self.x**b[0], self.y**b[1], self.z**b[2])
		else:
			raise ValueError('V3 error!')

	def __div__(self, b): # TODO, should i guard against div by zero?
		if type(b) == float or type(b) == int:
			return V3(self.x / b, self.y / b, self.z / b)
		elif len(b) == 3:
			return V3(self.x / b[0], self.y / b[1], self.z / b[2]) # This is matlab ./
		else:
			raise ValueError('V3 error!')

	def __rdiv__(self, b):
		if len(b) == 3:
			return V3(b[0] / self.x, b[1] / self.y, b[2] / self.z) # This is matlab ./
		else:
			raise ValueError('V3 error!')

	def __neg__(self):
		return V3(-self.x, -self.y, -self.z)
		
	def __eq__(self, b):
		if len(b) == 3:
			return self.x == b[0] and self.y == b[1] and self.z == b[2] # It is the same vector.
		else:
			raise ValueError('V3 error!')
	
	def __str__(self, d=3):
		return "(%.*f, %.*f, %.*f)" % (d, self.x, d, self.y, d, self.z)
#		return "(%f, %f, %f)" % (self.x, self.y, self.z)

	#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	#% V3 methods
	#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	def Copy(self, b):
		if len(b) == 3:
			self.x = b[0]
			self.y = b[1]
			self.z = b[2]
			return 1
		else:
			raise ValueError('V3 error!')

	def SqrMagnitude(self):
		return self.x**2 + self.y**2 + self.z**2

	def Magnitude(self):
		return math.sqrt(self.x**2 + self.y**2 + self.z**2)

	def Normalize(self):
		self_m = Magnitude(self);
		if self_m == 0:
			return V3(0, 0, 0)
		return V3(self.x / self_m, self.y / self_m, self.z / self_m)

	def Dot(self, b):
		return self.x * b[0] + self.y * b[1] + self.z * b[2]

	def Cross(self, b):
		return V3(self.y * b[2] - self.z * b[1], self.z * b[0] - self.x * b[2], self.x * b[1] - self.y * b[0])

	def Angle(self, b):
		den = (Magnitude(self) * Magnitude(b))
		if not den:
			return 0
		temp = Dot(self, b) / den
#		temp *= temp >= -1 and temp <= 1 # Don't know if I like these.
#		if not (temp >= -1 and temp <= 1):
#			raise ValueError('V3 error!')
		return math.acos(temp)

	def AngleSigned(self, b, around):
		den = (Magnitude(self) * Magnitude(b))
		if not den:
			return 0
		sign = (Dot(Cross(self, b), around) >= 0) * 2 - 1
		temp = Dot(self, b) / den
		return math.acos(temp) * sign

	def RotateY(self, a):
		rm = [	math.cos(a),	0,	math.sin(a), 
				0,				1,	0, 
				-math.sin(a),	0,	math.cos(a)]
		v2 = V3(0, 0, 0)
		v2[0] = self[0] * rm[0] + self[1] * rm[1] + self[2] * rm[2]
		v2[1] = self[0] * rm[3] + self[1] * rm[4] + self[2] * rm[5]
		v2[2] = self[0] * rm[6] + self[1] * rm[7] + self[2] * rm[8]
		return v2

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#% Vector functions alternative 1, faster.
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

def SqrMagnitude(a):
	return a[0]**2 + a[1]**2 + a[2]**2

def Magnitude(a):
	return math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)

def Normalize(a):
	a_m = Magnitude(a);
	if a_m == 0:
		return V3(0, 0, 0)
	return V3(a[0] / a_m, a[1] / a_m, a[2] / a_m)

def Dot(a, b):
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

def Cross(a, b):
	return V3(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

def Angle(a, b):
	den = (Magnitude(a) * Magnitude(b))
	if not den:
		return 0
	temp = Dot(a, b) / den
	temp *= temp >= -1 and temp <= 1
	return math.acos(temp)

def AngleSigned(a, b, around):
	den = (Magnitude(a) * Magnitude(b))
	if not den:
		return 0
	sign = (Dot(Cross(a, b), around) >= 0) * 2 - 1
	temp = Dot(a, b) / den
	temp *= temp >= -1 and temp <= 1
	return math.acos(temp) * sign

def RotateY(a, b):
	rm = [	math.cos(b),	0,	math.sin(b), 
			0,				1,	0, 
			-math.sin(b),	0,	math.cos(b)]
	v2 = V3(0, 0, 0)
	v2[0] = a[0] * rm[0] + a[1] * rm[1] + a[2] * rm[2]
	v2[1] = a[0] * rm[3] + a[1] * rm[4] + a[2] * rm[5]
	v2[2] = a[0] * rm[6] + a[1] * rm[7] + a[2] * rm[8]
	return v2

##################################################################
# Sign
##################################################################

def sign(a): # Lower case s in matlab -> less fuss this way.
	if type(a) == float or type(a) == int:
		return (a >= 0) * 2 - 1
	else:
		raise ValueError('V3 error!')



