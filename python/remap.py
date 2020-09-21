#! /usr/bin/python

from interpreter import *
import linuxcnc
import re
import math

from datetime import datetime
import time
from V3 import *

##################################################################

s = linuxcnc.stat()

#def g71_init_log(stringen, mode = 'a'):
#	with open('g71_init.log', mode) as tmplog:
#		tmplog.write("%s\n" % stringen)

#if not hal.component_exists("g71comp"):
#	g71_init_log("\nInit time is %s.\n" % (datetime.now().time()), 'w')
#	halcomp = hal.component("g71comp")
#	gremlin_io_pin = halcomp.newpin('gremlin-io', hal.HAL_S32, hal.HAL_IO)
#	halcomp.ready()
#	g71_init_log(halcomp)
##else:
##	g71_init_log(g71comp)

#g71_init_log("\nInit time2 is %s.\n" % (datetime.now().time()))

class Tool_seg(object):
	pos = V3(0, 0, 0)
	hat = V3(0, 0, -1) # * zdir

class Seg(object):
	type = 		-1
	start = 	V3(0, 0, 0)
	end = 	V3(0, 0, 0)
	radius = 	0
	center = 	V3(0, 0, 0)
	angleS = 	0
	angleE = 	0
	angleR = 	0
	fillet = 	0
	number = 	0
	f_code =	''
	s_code =	''
	m_code =	''
	
	def __str__(self):
		if self.type == 0 or self.type == 1:
			return("type = %d, start = %s, end = %s, fillet = %f, number = %d.\n" % (self.type, self.start, self.end, self.fillet, self.number))
		else:
			return("type = %d, start = %s, end = %s, radius = %f, center = %s, angleR = %f, fillet = %f, number = %d.\n" % (self.type, self.start, self.end, self.radius, self.center, self.angleR, self.fillet, self.number))

	def Gcode(self, e):
		if self.type == 0 or self.type == 1:
			return("g%d x%.*f z%.*f%s%s%s" % (self.type, e, self.end[0], e, self.end[2], self.f_code, self.s_code, self.m_code))
		elif self.type == 2 or self.type == 3:
			return("g%d x%.*f z%.*f r%.*f%s%s%s" % (self.type, e, self.end[0], e, self.end[2], e, self.radius, self.f_code, self.s_code, self.m_code))
		else:
			g71_log("Assert, Gcode write with invalid type = %d." % self.type)
			return 0

class G71_params:
	p = 0 # O-sub number.
	d = 0 # Cut depth.
	r = 0 # Retract height.
	i = 0 # X-allowance
	k = 0 # Z-allowance
	f = 0 # Stock removal feed-
	s = 0 # Stock removal spindle speed.
	j = 0 # 1 = constant surface velocity, 0 = fixed feed. 0.5 is often good.
	l = 0 # Dive angle, zero is straight down.
	t = 0 # Tool to use for finishing.
	e = 4 # Number of decimals in subs.
	fd = 0.33 # Dive feed is hard coded to one third of stockremoval feed.

def g71_log(stringen, mode = 'a'):
	with open('g71.log', mode) as tmplog:
		tmplog.write("%s\n" % stringen)

def write_sub(file_name, stringen, mode = 'a'): # mode w = overwrite, a = append.
	with open('../../nc_files/%s' % file_name, mode) as sub:
		sub.write("%s" % stringen)

##################################################################

def g71(self, **words):
	timestamp_dbg = time.time()
#	self.gremlin_io = 6 # Redraw Gremlin
	g71_log("\nTime is %s.\n" % (datetime.now().time()), 'w')
	write_sub('contour.ngc', 'o<contour> sub\n', 'w') # Same as the programmed path but with the fillet radii.
	write_sub('contour.ngc', 'g8 g90\n')
	write_sub('stock_removal.ngc', 'o<stock_removal> sub\n', 'w')
	write_sub('stock_removal.ngc', 'g8 g90\n')
	write_sub('roughing.ngc', 'o<roughing> sub\n', 'w')
	write_sub('roughing.ngc', 'g8 g90\n')
	write_sub('finishing.ngc', 'o<finishing> sub\n', 'w')
	write_sub('finishing.ngc', 'g8 g90\n')
	
	##################################################################
	# Get the g71 subroutine code.
	##################################################################

	s.poll()
	regex = r'\O0*%(p)i\s*SUB(.*)O0*%(p)i\s*ENDSUB' % words # \O0*71\s*SUB(.*)O0*71\s*ENDSUB
	gcode = re.findall(regex, open(s.file).read(), re.M | re.S | re.I)

	if len(gcode) == 0:
		g71_log("Error, G71 reference P%(p)i but O%(p)i SUB not in file.\n" % words) # G71 reference P71 but O71 SUB not in file
		return INTERP_ERROR

	gcode = gcode[0].split('\n')
#	g71_log("gcode = %s\n" % gcode) # ['', 'G0 X-1', 'G1 Z0', 'g1 x20 q2', ...

	##################################################################
	# Process the g71 subroutine code.
	##################################################################
	
	g7 = 1 + self.params['_lathe_diameter_mode'] # These are local changes until the end.
	g7_at_entry = g7
	g90 = self.params['_absolute']
	g90_at_entry = g90
	g91 = self.params['_incremental']
	g91_at_entry = g91
	g901 = self.params['_ijk_absolute_mode']
	g901_at_entry = g901
#	start_x = (s.position[0] - s.g5x_offset[0] - s.tool_offset[0] - s.g92_offset[0]) # * g7
#	start_z = (s.position[2] - s.g5x_offset[2] - s.tool_offset[2] - s.g92_offset[2])
	start_pos = V3(self.params['_X'], 0, self.params['_Z']) # These seem to be always radial.
	max_x = start_pos[0]
	min_x = start_pos[0]
#	g71_log("start pos = %s.\n" % start_pos)

	##################################################################
	# Initialize the segs.
	##################################################################

	g71_log("\nInfo, generating contour.\n")
	segs = []
	remove_last_seg = 0
	dofillet = 0
	n = -1
	for block in gcode:
		cmds = dict(re.findall('(\w)\s*([-+,\.\d]+)', block.lower(), re.S | re.I)) # TODO, this does not work with comments.
#		g71_log("G-code block = %s\n" % block) # G-code block = g1 x40 z-21 q5
#		g71_log("Block cmds = %s\n" % cmds) # Block cmds = {'q': '5', 'x': '40', 'z': '-21', 'g': '1'}
		
		if 'g' in cmds:
			if cmds['g'] == '91': # TODO, the regex does not read multiple 'g' on the same line. Works if g91 on separate line.
				g91 = 1
				g90 = 0
			elif cmds['g'] == '90':
				g91 = 0
				g90 = 1
	#		g71_log("g90 = %d, g91 = %d.\n" % (g90, g91))
			if cmds['g'] == '91.1':
				g901 = 0
			elif cmds['g'] == '90.1':
				g901 = 1
	#		g71_log("g901 = %d.\n" % g901)
			if cmds['g'] == '7':
				g7 = 2
			elif cmds['g'] == '8':
				g7 = 1
	#		g71_log("g7 = %d.\n" % g7)
		if not 'g' in cmds and ('x' in cmds or 'z' in cmds) or 'g' in cmds and cmds['g'] in ('0', '1', '2', '3', '00', '01', '02', '03'):
			n = n + 1 + dofillet
			segs.append(Seg())
			if dofillet:
				segs.append(Seg())
				segs[n-1].number = n-1
			segs[n].number = n
			if 'g' in cmds:
				segs[n].type = int(cmds['g'])
			else:
				segs[n].type = last_g
			last_g = segs[n].type
			
			if 'f' in cmds:
				segs[n].f_code = ' f%s' % cmds['f']
#				g71_log("segs[n].f_code = %s.\n" % segs[n].f_code)
#				g71_log("segs[n].Gcode(e) = %s.\n" % segs[n].Gcode(e))
			if 's' in cmds:
				segs[n].s_code = ' s%s' % cmds['s']
			if 'm' in cmds:
				segs[n].m_code = ' m%s' % cmds['m']
			
			if n == 0:
				segs[n].start = start_pos
				r_start = start_pos # Start point before fillet.
			else:
				segs[n].start = V3(segs[n-1-dofillet].end[0], 0, segs[n-1-dofillet].end[2])
			segs[n].end = V3(segs[n].start[0], 0, segs[n].start[2])
			
			if 'x' in cmds:
				segs[n].end[0] = (segs[n].end[0] + float(cmds['x']) / g7) * g91 + float(cmds['x']) / g7 * g90
			if 'z' in cmds:
				segs[n].end[2] = (segs[n].end[2] + float(cmds['z'])) * g91 + float(cmds['z']) * g90
			
			if segs[n].type > 1:
				if 'r' in cmds:
					segs[n].radius = float(cmds['r'])
					segs[n].center = Find_center(segs[n])
				else:
					if 'i' in cmds and 'k' in cmds:
						if g901:
							center = V3(float(cmds['i']), 0, float(cmds['k'])) # LinuxCNC does not acknowledge g7 for g90.1.
#							center = V3(float(cmds['i']) / g7, 0, float(cmds['k']))
						else:
							center = r_start + V3(float(cmds['i']), 0, float(cmds['k']))
					elif 'i' in cmds:
						if g901:
							center = V3(float(cmds['i']), 0, r_start[2])
#							center = V3(float(cmds['i']) / g7, 0, r_start[2])
						else:
							center = r_start + V3(float(cmds['i']), 0, 0)
					elif 'k' in cmds:
						if g901:
							center = V3(r_start[0], 0, float(cmds['k']))
						else:
							center = r_start + V3(0, 0, float(cmds['k']))
					else:
						g71_log("Error, I don't like the radius parameters in segment %d = %s.\n" % (n, segs[n].Gcode(3)))
						return INTERP_ERROR
					segs[n].radius = Magnitude(center - r_start)
					hubba = Cross(segs[n].end - r_start, center - r_start)
					segs[n].radius *= sign(hubba[1] * (segs[n].type - 2.5))
					segs[n].center = Find_center(segs[n])
					if Magnitude(center - segs[n].center) > .00001:
						g71_log("Warning, endpoint of arc is not on circle in segment %d = %s. Center error = %f.\n" % (n, segs[n].Gcode(3), Magnitude(center - segs[n].center))) # Maybe refuse here instead, but I leave it up to the user to read the log before executing.
			
#			g71_log("segs[n].center = %s." % segs[n].center)
#			g71_log("segs[n].radius = %f.\n" % segs[n].radius)
			r_start	= segs[n].end # start before fillet.
			[segs[n].angleS, segs[n].angleE, segs[n].angleR] = Find_angles(segs[n])
#			g71_log("Find_angles(segs[n]) = %.2f, %.2f, %.2f." % (segs[n].angleS, segs[n].angleE, segs[n].angleR))
			if 'q' in cmds:
				segs[n].fillet = float(cmds['q'])
			
#			max_x = max(max_x, segs[n].end[0])
#			min_x = min(min_x, segs[n].end[0])
#			# TODO, check also if a radius is poking out, not only the end points.
#			if g71_params.d < 0 and segs[n].end[0] > start_pos[0] or g71_params.d > 0 and segs[n].end[0] < start_pos[0]:
#				g71_log("Warning, Profile extending above start position in X-axis. Segment %d, X = %f.\n" % (n, segs[n].end[0]))
##				return INTERP_ERROR # TODO, make this explain also on screen, not only in the log.

			##################################################################
			# Add the fillets
			##################################################################
			
			if dofillet and n > 1: # Note that this is if the last segment had a fillet.
				if Do_fillet(segs[n-2], segs[n-1], segs[n]):
					segs[n-2].end = segs[n-1].start
					if segs[n-2].type > 1:
						[segs[n-2].angleS, segs[n-2].angleE, segs[n-2].angleR] = Find_angles(segs[n-2])
						segs[n-2].radius = abs(segs[n-2].radius) * sign(math.pi - abs(segs[n-2].angleR))
				else: # Programmer wanted impossible fillet.
					n = n - 1
					del(segs[-1])
			[segs[n].angleS, segs[n].angleE, segs[n].angleR] = Find_angles(segs[n])
			segs[n].radius = abs(segs[n].radius) * sign(math.pi - abs(segs[n].angleR))
			dofillet = abs(segs[n].fillet) > 0

	##################################################################
	# Get the g71 parameters (called words here). Set default if parameter not present.
	##################################################################

	zdir = ((segs[-1].end[2] - segs[0].start[2]) < 0) * 2 - 1 # 1 = tool is moving in the normal Z direction, -1 = tool is not.
	xdir = ((segs[0].end[0] - segs[0].start[0]) < 0) * 2 - 1 # 1 = tool is working towards the center, -1 = inside turning.
	if not (xdir and zdir):
		g71_log("Error. First move must have x movement. Program must have range in the Z-direction. xdir = %d, zdir = %d.\n" % (xdir, zdir))
		return INTERP_ERROR
	
#	g71_log("words %s" % words) # {'d': 4.0, 'f': 200.0, 'i': 2.0, 'j': 0.5, 'p': 71.0, 's': 400.0, 'r': 1.0, 't': 2.0}
	g71_params = G71_params()
	
	if 'd' in words:
		g71_params.d = abs(words['d'])
		g71_params.d *= -xdir
	else:
		g71_log("Error, no g71 parameter d.\n")
		return INTERP_ERROR
		
	if 'r' in words:
		g71_params.r = abs(words['r']) * -sign(g71_params.d)
	else:
		g71_params.r = -g71_params.d
		        
	if 'i' in words:
		g71_params.i = abs(words['i']) * -sign(g71_params.d)
	else:
		g71_params.i = -g71_params.d / 3
		        
	if 'k' in words:
		g71_params.k = abs(words['k']) * -sign(g71_params.d) * zdir
	else:
		g71_params.k = 0 # -g71_params.d / 20 * zdir
		        
	if 'j' in words: # 0 = no csv feed .5 = 50 % csv feed. 
		g71_params.j = abs(words['j'])
		if g71_params.j > 1: g71_params.j = 1
	else:
		g71_params.j = .5
		
	if 'l' in words: # Dive angle.
		g71_params.l = int(abs(words['l']))
		l = g71_params.l
	else:
		l = 0 # Dive angle defaults to straight down.

	if 'f' in words: g71_params.f = words['f']
	if 's' in words: g71_params.s = words['s']
	if 't' in words: g71_params.t = words['t']

	g71_params.d = g71_params.d / g7
	g71_params.r = g71_params.r / g7
	g71_params.i = g71_params.i / g7

	e = g71_params.e # Number of decimals in subs is now hard coded.
	fd = g71_params.fd * g71_params.f # Dive feed.

	###########################################################################

	write_sub('contour.ngc', "g0 x%.*f z%.*f\n" % (e, segs[0].start[0], e, segs[0].start[2]))
	for seg in segs:
		write_sub('contour.ngc', '%s\n' % seg.Gcode(e))
	if segs[n].end[0] * -sign(g71_params.d) < start_pos[0] * -sign(g71_params.d): # Add closing segment.
		write_sub('contour.ngc', "g1 x%.*f z%.*f\n" % (e, start_pos[0], e, segs[-1].end[2]))
	write_sub('contour.ngc', "g0 x%.*f z%.*f\n" % (e, start_pos[0], e, start_pos[2]))

	###########################################################################

#	g71_log("self.params['_current_tool'] = %d.\n" % self.params['_current_tool'])
	tool_vector = Get_tool_vector(self.params['_current_tool'])
	tool_radius = tool_vector[1]
	tool_vector[1] = 0;
#	g71_log('tool_vector = %s.\n' % tool_vector)
#	g71_log('tool_radius = %s.\n' % tool_radius)
	
	##################################################################
	# Find the offsetted tool paths.
	##################################################################
	
#	g71_log("self.params['_feed'] = %f\n" % self.params['_feed'])
#	g71_log("self.params['_rpm'] = %f\n" % self.params['_rpm'])
	royal_feed = self.params['_feed']
	last_feed = royal_feed

	for nn in ['finishing', 'roughing']:
		if nn == 'finishing':
			g71_log("\nInfo, offsetting finishing path.\n")
			if g71_params.t: # Use a different tool for finishing.
				tool_vector = Get_tool_vector(g71_params.t)
				write_sub('%s.ngc' % nn, "g91 g28 x0\nt%d m6 g43\nm3\ng90 g0 x%.*f z%.*f\n" % (g71_params.t, e, start_pos[0], e, start_pos[2]))
			else:
				tool_vector = Get_tool_vector(self.params['_current_tool'])
			tool_radius = tool_vector[1]
			tool_vector[1] = 0;
		#	g71_log('ftool_vector = %s.\n' % tool_vector)
		#	g71_log('ftool_radius = %s.\n' % tool_radius)
			K = 0 # Save for finishing in Y.
			I = 0 # Save for finishing in X.
		else:
			g71_log("\nInfo, offsetting roughing path.\n")
			tool_vector = Get_tool_vector(self.params['_current_tool'])
			tool_radius = tool_vector[1]
			tool_vector[1] = 0;
		#	g71_log('rtool_vector = %s.\n' % tool_vector)
		#	g71_log('rtool_radius = %s.\n' % tool_radius)
			D = g71_params.d
			R = g71_params.r
			I = g71_params.i
			K = g71_params.k

		u = [I - abs(K) * sign(I), 0, 0] + tool_vector
		offset = tool_radius * -sign(g71_params.d) * zdir + K
		
#	###########################################################################
		segs_offset = []
		skipped = 0
		bad = 0
		n = 0

		while not n + skipped == len(segs):
			if not bad:
				segs_offset.append(Seg()) # I consider redoing g71 with the segments as a linked list.
				segs_offset[n].number = n # TODO, the numbers are not correct everywhere.
	#			g71_log("n = %d. segs_offset[n].number = %d. len(segs_offset) = %d.\n" % (n, segs_offset[n].number, len(segs_offset)))
			else:
				g71_log("Info, removed contour segment %d while offsetting the path (contour code = %s).\n" % (n + skipped - 1, segs[n + skipped - 1].Gcode(e)))
			Seg_copy(segs_offset[n], segs[n + skipped])
			if segs_offset[n].type == 0 or segs_offset[n].type == 1: # TODO, I can make this smarter and avoid overcut.
				seg_hat = Normalize(segs_offset[n].end - segs_offset[n].start)
				offset_v = Cross(seg_hat, [0, 1, 0]) * offset
				segs_offset[n].start = segs_offset[n].start + offset_v
				segs_offset[n].end = segs_offset[n].end + offset_v
			
				if not len(segs_offset[n].f_code): # Reset feed after constant surface feed.
					if not (royal_feed == last_feed):
						segs_offset[n].f_code = ' f%.*f' % (e, royal_feed)
						last_feed = royal_feed
				else:
					royal_feed = float(segs_offset[n].f_code[2:])
					last_feed = royal_feed
			else: # This too.
				offset_v = Normalize(segs_offset[n].start - segs_offset[n].center) * offset * 2 * (segs_offset[n].type - 2.5)
				segs_offset[n].start = segs_offset[n].start + offset_v
				offset_v = Normalize(segs_offset[n].end - segs_offset[n].center) * offset * 2 * (segs_offset[n].type - 2.5)
				segs_offset[n].end = segs_offset[n].end + offset_v
				segs_offset[n].radius = Magnitude(segs_offset[n].center - segs_offset[n].end) * sign(segs_offset[n].radius)
				
				if segs_offset[n].radius == 0:
					tmp = n + skipped - 1
					g71_log("Warning, contour segment %d has zero radius (contour code = %s).\n" % (tmp, segs[tmp].Gcode(e)))
				elif g71_params.j > 0:
					if not len(segs_offset[n].f_code): # Constant surface feed.
						tsign = sign(abs(segs_offset[n].radius) - abs(segs[n + skipped].radius))
						csv_feed = royal_feed * (1 - g71_params.j + g71_params.j * abs((abs(segs_offset[n].radius) + tool_radius * tsign) / segs_offset[n].radius))
						segs_offset[n].f_code = ' f%.*f' % (e, csv_feed)
						last_feed = csv_feed
					else:
						royal_feed = float(segs_offset[n].f_code[2:])
						tsign = sign(abs(segs_offset[n].radius) - abs(segs[n + skipped].radius))
						csv_feed = royal_feed * (1 - g71_params.j + g71_params.j * abs((abs(segs_offset[n].radius) + tool_radius * tsign) / segs_offset[n].radius))
						segs_offset[n].f_code = ' f%.*f' % (e, csv_feed)
						last_feed = csv_feed
			
			bad = Dot(segs_offset[n].end - segs_offset[n].start, segs[n + skipped].end - segs[n + skipped].start) <= .0001 # A radius was inverted.
			skipped = skipped + bad
			n = n + (bad == 0)

		for n in range(len(segs_offset) - 1):
			intersect1 = Find_intersect(segs_offset[n], segs_offset[n+1])
			segs_offset[n].end = intersect1
			segs_offset[n+1].start = intersect1
			[segs_offset[n].angleS, segs_offset[n].angleE, segs_offset[n].angleR] = Find_angles(segs_offset[n])
			if abs(segs_offset[n].angleR) > math.pi:
				segs_offset[n].radius = -abs(segs_offset[n].radius)

		if u:
			for n in range(len(segs_offset)):
				segs_offset[n].end = segs_offset[n].end + u
				segs_offset[n].start = segs_offset[n].start + u
				segs_offset[n].center = segs_offset[n].center + u

		if segs_offset[n].end[0] * -sign(g71_params.d) < start_pos[0] * -sign(g71_params.d): # Add closing segment.
			g71_log('Info, adding a closing segment to the path.\n')
			n = n + 1
			segs_offset.append(Seg())
			segs_offset[n].number = n
			segs_offset[n].type = 1
			segs_offset[n].start = segs_offset[n-1].end
			segs_offset[n].end = V3(start_pos[0], 0, segs_offset[n].start[2])
			segs_offset[n].f_code = ' f%.*f' % (e, royal_feed)

		if (segs_offset[0].type == 0 or segs_offset[0].type == 1) and not segs_offset[0].start[0] == start_pos[0]:
			intersect1 = []
			intersect2 = []
			seg1_hat = [0,0,-1]
			seg2 = segs_offset[0]
			cl_v = Cross(seg1_hat, Cross(seg1_hat, seg2.start - start_pos))
			cl_v_m = Magnitude(cl_v)
			seg2_hat = Normalize(seg2.end - seg2.start)
			den = Dot(Normalize(cl_v), seg2_hat)
			if den == 0 or cl_v_m == 0:
				g71_log('Error, first move of g71 does not move in X-axis.\n')
				return INTERP_ERROR
			else:
				segs_offset[0].start = seg2.start + seg2_hat * cl_v_m / den
		elif not segs_offset[0].start[0] == start_pos[0]:
			g71_log('Warning, first move of g71 is not G0 or G1.\n')

		write_sub('%s.ngc' % nn, "g0 x%.*f z%.*f\n" % (e, segs_offset[0].start[0], e, segs_offset[0].start[2])) # Use g0 to make manual approach less scary (no g0 jump after g1).
#		write_sub('%s.ngc' % nn, "g1 x%.*f z%.*f\n" % (e, segs_offset[0].start[0], e, segs_offset[0].start[2]))
		for seg in segs_offset:
			write_sub('%s.ngc' % nn, '%s\n' % seg.Gcode(e))
		write_sub('%s.ngc' % nn, "g0 x%.*f z%.*f\n" % (e, start_pos[0], e, start_pos[2]))

	###########################################################################
	# Generate stock removal tool path.
	###########################################################################

	g71_log("\nInfo, generating stock removal path.\n")
	seg_rough = segs_offset

	firstX = start_pos[0] # + .000001111111 * -sign(D) # Fuling
	tool_seg = Tool_seg()
	tool_seg.pos.Copy(start_pos) # This works,
#	tool_seg.pos = start_pos # this does not. Be awake, it can bite so hard.
	tool_seg.pos[0] = firstX - D # Becomes firstX in the while loop.
	
	intersects = []
	cut = -1
	done = 0
	while not done:
		intersects.append([])
		cut = cut + 1
		tool_seg.pos[0] += D
		done = 1
		
		for n in range(len(seg_rough)):
			[intersect1, intersect2] = Find_tool_intersect(tool_seg, seg_rough[n], zdir) # Have the intersects in the right order.

			if not len(intersect1) and not len(intersect2):
				continue
			else:
				if len(intersect1):
					if not (seg_rough[n-1].start[0] == seg_rough[n-1].end[0] and seg_rough[n-1].start[0] == intersect1[0]): # Remove intersect if it comes after a horizontal segment. Seems to work.
						intersects[cut].append(intersect1)
						done = 0
				if len(intersect2):
					if not (seg_rough[n-1].start[0] == seg_rough[n-1].end[0] and seg_rough[n-1].start[0] == intersect2[0]):
						intersects[cut].append(intersect2)
						done = 0

		if len(intersects[cut]) == 1:
			g71_log('Warning, only one intersect[%d][0] = %s.\n' % (cut, intersects[cut][0]))
#			return INTERP_ERROR
			intersects[cut].append(intersects[cut][0])
			g71_log('Patched it.\n')

		for k in range(len(intersects[cut]) - 1):
			if cut > 0 and intersects[cut][k][0] == intersects[cut][k+1][0] and intersects[cut][k][2] == intersects[cut][k+1][2]:
				g71_log('Found double intersect = %s.\n' % intersects[cut][k+1])
				for p in range(k+1, len(intersects[cut]) - 1):
					intersects[cut][p] = intersects[cut][p+1]
				del(intersects[cut][-1])
				g71_log('Fixed it.\n')
			if k > len(intersects[cut]) - 3:
				break
#		else:
		if len(intersects[cut]) % 2: # Need to take this seriously.
			g71_log('Error, uneven number of intersects in cut %d.\n' % cut )
			for n in range(len(intersects[cut])):
				g71_log('intersect[%d][%d] = %s.\n' % (cut, n, intersects[cut][n]))

	if len(intersects[0]) == 0:
		g71_log('Assert, no intersects in cut 0.') # Should never happen.
		return INTERP_ERROR
	
	# Check if stock removal is needed. (Patch, hope it does not mess up something else.)
	if len(intersects[1]) == 0:
		g71_log('No intersects in cut 1. No stock removal generated.') # Can happen if there is no stock removal.
	else:
			
		# Use only first and last intersets of the first line at X = start_pos[0]. Yes, it may crash if stock protrudes higher than start X.
		intersects[0][0] = start_pos
		intersects[0][1] = intersects[0][-1] # The zeroeth level is only a faked, virtual cut.
		del intersects[0][2:]
	#	g71_log(intersects)
	#	for m in range(0, len(intersects[0:])):
	#		for n in range(len(intersects[m])):
	#			g71_log('intersect[%d][%d] = %s.\n' % (m, n, intersects[m][n]))

		pocket_stack = []
		write_sub('stock_removal.ngc', 'f%.*f s%.*f\n' % (e, g71_params.f, e, g71_params.s))
		write_sub('stock_removal.ngc', 'G0 Z%.*f\n' % (e, intersects[1][0][2])) # Z feed to above first intersect.
		# Use g0 to make manual approach less scary (no g0 jump after g1).
#		write_sub('stock_removal.ngc', 'G1 Z%.*f\n' % (e, intersects[1][0][2])) # Z feed to above first intersect.

		ints_done = [0] * len(intersects)  # Number of intersections on depth n = len(intersects[n]).
		ints_done[0] = 2 # Because the zeroeth level is only a faked, virtual cut. It is already faked, i.e., done.
		levels_done = 0 # Number of depth levels = len(intersects) - 1. Last level has no intersects.
		this_level_done = 0 # Flag
		done = 0 # Flag
		n = 0
		while not done and len(intersects) > 2:
	 		n = n + 1 # Current depth level.
			this_level_done = 0
			m = ints_done[n] - 1 # Current intersect.
			while not this_level_done:
				m = m + 1
				if m % 2 == 0: # At the start of a cut.
					if n > len(intersects) - 1 or m > len(intersects[n]) - 1: # There was an error.
						g71_log('Assert, in generating stock removal tool path. n = %d, m = %d.\n' % (n, m))
						g71_log('len(intersects[n]) = %d, ints_done[n] = %d, levels_done = %d.\n' % (len(intersects[n]), ints_done[n], levels_done))
						g71_log(intersects)
						done = 1
						break
					next_pos = intersects[n][m]
					write_sub('stock_removal.ngc', 'G1 X%.*f Z%.*f\n' % (e, next_pos[0] - g71_params.d, e, next_pos[2])) # feed down.
					write_sub('stock_removal.ngc', 'G1 X%.*f Z%.*f f%.*f\n' % (e, next_pos[0], e, next_pos[2] + g71_params.d * math.sin(g71_params.l * math.pi / 180) * zdir * xdir, e, fd)) # feed down with dive angle.
				elif m % 2 == 1: # At the end of a cut.
					if n > len(intersects) - 1 or m > len(intersects[n]) - 1: # There was an error.
						g71_log('Assert, in generating stock removal tool path. n = %d, m = %d.\n' % (n, m))
						g71_log('len(intersects[n]) = %d, ints_done[n] = %d, levels_done = %d.\n' % (len(intersects[n]), ints_done[n], levels_done))
						g71_log(intersects)
						done = 1
						break
					next_pos = intersects[n][m]
					write_sub('stock_removal.ngc', 'G1 X%.*f Z%.*f f%.*f\n' % (e, next_pos[0], e, next_pos[2], e, g71_params.f)) # Z feed.
					next_pos = next_pos + [R, 0, 0]
					write_sub('stock_removal.ngc', 'G1 X%.*f Z%.*f\n' % (e, next_pos[0], e, next_pos[2])) # Feed up.
					if m == len(intersects[n]) - 1 and n-1 == levels_done: # This level is all done.
						this_level_done = 1
						levels_done = n
					ints_done[n] = m + 1 # m + 1 intersections in depth n are done.
					if len(intersects[n]) > ints_done[n] and intersects[n][m+1][2] * zdir > intersects[n-1][ints_done[n-1]-1][2] * zdir: # If this depth level is not done, and next point on this level is to the right of the done point on the last level.
	#					g71_log('Save pocket, n = %d, m = %d\n' % (n, m))
						pocket_stack.append([n, m])
					if len(intersects[n+1]) > ints_done[n+1] and intersects[n+1][ints_done[n+1]][2] * zdir > next_pos[2] * zdir: # If next depth has intersections, and has a not done point to the right.
	#					g71_log('Go back, n = %d, m = %d\n' % (n, m))
						next_pos = V3(next_pos[0], 0, intersects[n+1][ints_done[n+1]][2])
						write_sub('stock_removal.ngc', 'G0 X%.*f Z%.*f\n' % (e, next_pos[0], e, next_pos[2])) # Rapid Z to first intersect of next depth.
						this_level_done = 1
					elif len(pocket_stack): # Go to next intersection.
	#					g71_log('Next pocket, n = %d, m = %d\n' % (n, m))
						n = pocket_stack[-1][0]
						m = pocket_stack[-1][1]
						del pocket_stack[-1]
						next_pos = V3(intersects[n][m+1][0] - D + R, 0, next_pos[2])
						write_sub('stock_removal.ngc', 'G0 X%.*f Z%.*f\n' % (e, next_pos[0], e, next_pos[2])) # Rapid to clear level X.
						next_pos = V3(next_pos[0], 0, intersects[n][ints_done[n]][2])
						write_sub('stock_removal.ngc', 'G0 X%.*f Z%.*f\n' % (e, next_pos[0], e, next_pos[2])) # Rapid Z to next intersection.
					else: # Go to start position.
						next_pos = [start_pos[0], 0, next_pos[2]]
						write_sub('stock_removal.ngc', 'G0 X%.*f Z%.*f\n' % (e, next_pos[0], e, next_pos[2]))
						next_pos = [next_pos[0], 0, start_pos[2]]
						write_sub('stock_removal.ngc', 'G0 X%.*f Z%.*f\n' % (e, next_pos[0], e, next_pos[2]))
						done = 1

	##################################################################
	# End the o-code files. Restore to entry mode.
	##################################################################
	
	g8s = 'g8'
	if g7_at_entry == 2:
		g8s = 'g7'
	write_sub('stock_removal.ngc', '%s ' % g8s)
	write_sub('contour.ngc', '%s ' % g8s)
	write_sub('roughing.ngc', '%s ' % g8s)
	write_sub('finishing.ngc', '%s ' % g8s)
	g91s = 'g91'
	if g90_at_entry:
		g91s = 'g90' 
	write_sub('stock_removal.ngc', '%s ' % g91s)
	write_sub('contour.ngc', '%s ' % g91s)
	write_sub('roughing.ngc', '%s ' % g91s)
	write_sub('finishing.ngc', '%s ' % g91s)
	g911s = 'g91.1'
	if g901_at_entry:
		g911s = 'g90.1' 
	write_sub('stock_removal.ngc', '%s ' % g911s)
	write_sub('contour.ngc', '%s ' % g911s)
	write_sub('roughing.ngc', '%s ' % g911s)
	write_sub('finishing.ngc', '%s ' % g911s)

	write_sub('contour.ngc', '\no<contour> endsub\n')
	write_sub('stock_removal.ngc', '\no<stock_removal> endsub\n')
	write_sub('roughing.ngc', '\no<roughing> endsub\n')
	write_sub('finishing.ngc', '\no<finishing> endsub\n')

	delta_time = time.time() - timestamp_dbg
	g71_log("\ng71 load time %f seconds." % delta_time)

###########################################################################
# Get_tool_vector
###########################################################################

def Get_tool_vector(toolnr):
	tool_radius = 0
	tool_dir = 0

	for tool in s.tool_table:
#		g71_log("tool.id = %d" % tool.id)
		if tool.id == toolnr:
			tool_radius = tool.diameter / 2
			tool_dir = tool.orientation
			break
	else:
		g71_log("Warning, tool nr %d not valid for radius compensation.\n" % toolnr)
		return V3(0, 0, 0)

	tool_radius = tool.diameter / 2
	tool_dir = tool.orientation
#	g71_log("tool.id = %d, tool radius = %f, tool_dir = %d.\n" % (toolnr, tool_radius, tool_dir))
	
	if tool_dir == 9:
		tool_vector_Z = 0
		tool_vector_x = 0
		return V3(tool_vector_Z, tool_radius, tool_vector_x)

	alfa = tool_dir * -90 + 45
	alfa = alfa + 45 * (tool_dir > 4)
	tv_m = tool_radius * (1 + ((alfa % 90) == 45) * (math.sqrt(2) - 1))
	tool_vector_z = tv_m * math.cos(alfa * math.pi / 180)
	tool_vector_x = tv_m * math.sin(alfa * math.pi / 180)
#	g71_log("tool_vector = [%.3f, 0, %.3f].\n" % (tool_vector_x, tool_vector_z))
	return V3(tool_vector_x, tool_radius, tool_vector_z)

###########################################################################
# Find_tool_intersect
###########################################################################

#function [intersect1, intersect2] = 
def Find_tool_intersect(seg1, seg2, zdir):
	# seg1 is tool path
	if seg2.type == 0 or seg2.type == 1:
		intersect1 = []
		intersect2 = []
		if (seg2.start[0] <= seg1.pos[0] and seg2.end[0] >= seg1.pos[0]) == (seg2.end[0] <= seg1.pos[0] and seg2.start[0] >= seg1.pos[0]):
			return [[], []] # Previous line is, if not a xor b. 
		cl_v = Cross(seg1.hat, Cross(seg1.hat, seg2.start - seg1.pos))
		cl_v_m = Magnitude(cl_v)
		seg2_hat = Normalize(seg2.end - seg2.start)
		den = Dot(Normalize(cl_v), seg2_hat)
		if den == 0 and not cl_v_m == 0:
			intersect1 = []
		elif cl_v_m == 0:
			intersect1 = seg2.start
		else:
			intersect1 = seg2.start + seg2_hat * cl_v_m / den

	elif seg2.type == 2 or seg2.type == 3:
		cl_v = Cross(seg1.hat, seg2.center - seg1.pos) # TODO, maybe a smarter vector length can improve resolution.
		cl_v_m2 = SqrMagnitude(cl_v)
		cl_v = Cross(seg1.hat, cl_v)
		i_dist = seg2.radius**2 - cl_v_m2
		if i_dist < 0:
			return [[], []]
		else:
			i_dist = math.sqrt(i_dist)
		
		intersect1 = seg2.center + cl_v - seg1.hat * i_dist
		intersect2 = seg2.center + cl_v + seg1.hat * i_dist
	
		i_v1 = intersect1 - seg2.center
		i_angle = math.atan2(i_v1[0], i_v1[2])
		if seg2.type == 3:
			if not ((i_angle >= seg2.angleS and i_angle <= seg2.angleS + seg2.angleR) or (i_angle <= seg2.angleE and i_angle >= seg2.angleE - seg2.angleR)):
				intersect1 = []
		elif not ((i_angle <= seg2.angleS and i_angle >= seg2.angleS + seg2.angleR) or (i_angle >= seg2.angleE and i_angle <= seg2.angleE - seg2.angleR)):
				intersect1 = []
	
		i_v1 = intersect2 - seg2.center
		i_angle = math.atan2(i_v1[0], i_v1[2])
		if seg2.type == 3:
			if not ((i_angle >= seg2.angleS and i_angle <= seg2.angleS + seg2.angleR) or (i_angle <= seg2.angleE and i_angle >= seg2.angleE - seg2.angleR)):
				intersect2 = []
		elif not ((i_angle <= seg2.angleS and i_angle >= seg2.angleS + seg2.angleR) or (i_angle >= seg2.angleE and i_angle <= seg2.angleE - seg2.angleR)):
				intersect2 = []
	
		# This will need to reverse (gt -> lt) if tool is moving "backward" in Z.
		if len(intersect1) and len(intersect2) and zdir * intersect1[2] < zdir * intersect2[2]:
			temp = intersect1
			intersect1 = intersect2
			intersect2 = temp
			
	return [intersect1, intersect2]

##################################################################
# Do_fillet
##################################################################
	
def Do_fillet(seg1, fillet, seg2): # Be aware, this seems to be like pass by reference!
	if (seg1.type == 0 or seg1.type == 1) and (seg2.type == 0 or seg2.type == 1):
		temp_v = Cross(seg1.end - seg1.start, seg2.end - seg2.start)
		if Magnitude(temp_v) == 0:
			Seg_copy(fillet, seg2) # i.e., remove the fillet.
			fillet.number = seg2.number - 1
			g71_log('* Warning, no fillet possible after contour segment %d.\n' % seg1.number)
			return 0
			
		r1_v = Normalize(Cross(seg1.end - seg1.start, temp_v)) * abs(seg1.fillet)
		r2_v = Normalize(Cross(seg2.end - seg2.start, temp_v)) * abs(seg1.fillet)
		l1_v = (r1_v + r2_v) / 2
		l1_v_hat = Normalize(l1_v)
		l1_v_m = Magnitude(l1_v)
		l = seg1.fillet**2 / l1_v_m
		l_v = l1_v_hat * l
#		Seg_copy(fillet, seg2)
		fillet.type = 2 + (temp_v[1] > 0)
		fillet.fillet = 0
		fillet.radius = seg1.fillet
		fillet.center = seg1.end - l_v
		fillet.start = fillet.center + r1_v
		fillet.end = fillet.center + r2_v
		[fillet.angleS, fillet.angleE, fillet.angleR] = Find_angles(fillet)
		seg2.start = fillet.end
		return 1
		
	elif (seg1.type == 0 or seg1.type == 1) and (seg2.type == 2 or seg2.type == 3):
		seg1.hat = Normalize(seg1.end - seg1.start)
		e_line = seg1.end - seg2.center
		seg1_p = Cross(seg1.hat, [0, 1, 0])
		fix_sign = sign(Dot(Cross([0, 1, 0], e_line), seg1_p)) * (seg2.type - 2.5) * 2
		fix_sign2 = -sign(Dot(seg1.hat, e_line))
		seg1_p = seg1_p * fix_sign
		seg1_p_hat = Normalize(seg1_p)
	
		cl_v = Cross(seg1.hat, seg2.center - (seg1.end + seg1_p_hat * abs(seg1.fillet)))
		cl_v_m2 = SqrMagnitude(cl_v)
		cl_v = Cross(seg1.hat, cl_v)
		i_dist_sqr = (abs(seg2.radius) + abs(seg1.fillet) * fix_sign2)**2 - cl_v_m2
		if i_dist_sqr < 0:
			i_dist = 0
			if i_dist_sqr < -.0001:
				Seg_copy(fillet, seg2) # i.e., remove the fillet.
				fillet.number = seg2.number - 1
				g71_log('** Warning, no fillet possible after contour segment %d.\n' % seg1.number)
				return 0
		else:
			i_dist = math.sqrt(i_dist_sqr)
	
		intersect1 = seg2.center + cl_v + seg1.hat * i_dist
		intersect2 = seg2.center + cl_v - seg1.hat * i_dist
		if Magnitude(seg2.start - intersect2) < Magnitude(seg2.start - intersect1):
			intersect1 = intersect2
	
#		Seg_copy(fillet, seg2)
		temp_v = Cross(e_line, seg1.hat)
		fillet.type = 2 + (fix_sign < 0)
		fillet.fillet = 0
		fillet.radius = seg1.fillet
		fillet.center = intersect1
		fillet.start = fillet.center - seg1_p_hat * abs(seg1.fillet)
		fillet.end = seg2.center + Normalize(fillet.center - seg2.center) * abs(seg2.radius)
		if fillet.radius < 0:
			point = fillet.start + (fillet.end - fillet.start) / 2
			fillet.center = point - (fillet.center - point)

		[fillet.angleS, fillet.angleE, fillet.angleR] = Find_angles(fillet)
		seg2.start = fillet.end
		return 1

	elif (seg1.type == 2 or seg1.type == 3) and (seg2.type == 0 or seg2.type == 1):
		seg2_hat = Normalize(seg2.start - seg2.end)
		e_line = seg2.start - seg1.center
		seg2_p = Cross(seg2_hat, [0, 1, 0])
		fix_sign = sign(Dot(Cross([0, 1, 0], e_line), seg2_p)) * -(seg1.type - 2.5) * 2
		fix_sign2 = -sign(Dot(seg2_hat, e_line))
		seg2_p = seg2_p * fix_sign
		seg2_p_hat = Normalize(seg2_p)

		cl_v = Cross(seg2_hat, seg1.center - (seg2.start + seg2_p_hat * abs(seg1.fillet)))
		cl_v_m2 = SqrMagnitude(cl_v)
		cl_v = Cross(seg2_hat, cl_v)
		i_dist_sqr = (abs(seg1.radius) + abs(seg1.fillet) * fix_sign2)**2 - cl_v_m2
		if i_dist_sqr < 0:
			i_dist = 0
			if i_dist_sqr < -.0001:
				Seg_copy(fillet, seg2) # i.e., remove the fillet.
				fillet.number = seg2.number - 1
				g71_log('** Warning, no fillet possible after contour segment %d.\n' % seg1.number)
				return 0
		else:
			i_dist = math.sqrt(i_dist_sqr)

		intersect1 = seg1.center + cl_v + seg2_hat * i_dist
		intersect2 = seg1.center + cl_v - seg2_hat * i_dist
		if Magnitude(seg1.end - intersect2) < Magnitude(seg1.end - intersect1):
			intersect1 = intersect2

#		Seg_copy(fillet, seg2)
		temp_v = Cross(e_line, seg2_hat)
		fillet.type = 2 + (fix_sign > 0)
		fillet.fillet = 0
		fillet.radius = seg1.fillet
		fillet.center = intersect1
		fillet.start = seg1.center + Normalize(fillet.center - seg1.center) * abs(seg1.radius)
		fillet.end = fillet.center - seg2_p_hat * abs(seg1.fillet)
		if fillet.radius < 0:
			point = fillet.start + (fillet.end - fillet.start) / 2
			fillet.center = point - (fillet.center - point)

		[fillet.angleS, fillet.angleE, fillet.angleR] = Find_angles(fillet)
		seg2.start = fillet.end
		return 1

	elif (seg1.type == 2 or seg1.type == 3) and (seg2.type == 2 or seg2.type == 3): # and seg1.type ~= seg2.type
		c_line = seg2.center - seg1.center
		c_line_m = Magnitude(c_line)
		koll = c_line_m - (abs(seg1.radius) + abs(seg2.radius))
		if koll > 0:
			c_line_m = abs(seg1.radius) + abs(seg2.radius)
			if koll < -.0001:
				Seg_copy(fillet, seg2) # i.e., remove the fillet.
				fillet.number = seg2.number - 1
				g71_log('*** Warning, no fillet possible after contour segment %d.\n' % seg1.number)
				return 0
		c_line_hat = Normalize(c_line)
		e_line = seg1.end - seg1.center
		ce_cross = Cross(c_line, e_line)
		fix_sign = -((ce_cross[1] <= 0) * 2 - 1) * ((seg1.type < seg2.type) * 2 - 1)
		fix_sign2 = -(seg1.type == seg2.type)
		if fix_sign2:
			fix_sign = 1
		else:
			fix_sign2 = 1

		l1 = (c_line_m**2 + (abs(seg1.radius) - abs(seg1.fillet) * fix_sign * fix_sign2)**2 - (abs(seg2.radius) + abs(seg1.fillet) * fix_sign)**2) / 2 / c_line_m
		c_point = seg1.center + c_line_hat * l1
		i_dist_sqr = (abs(seg1.radius) - abs(seg1.fillet) * fix_sign * fix_sign2)**2 - l1**2
		if i_dist_sqr < 0:
			i_dist = 0
			if i_dist_sqr < -.0001:
				Seg_copy(fillet, seg2) # i.e., remove the fillet.
				fillet.number = seg2.number - 1
				g71_log('**** Warning, no fillet possible after contour segment %d.\n' % seg1.number)
				return 0
		else:
			i_dist = math.sqrt(i_dist_sqr)
		i_line_hat = Cross(c_line_hat, [0, -1, 0])
#		i_line_hat = RotateY(c_line_hat, math.pi / 2)
		i_line = i_line_hat * i_dist
	
		if ce_cross[1] < 0:
			i_line = -i_line
	
#		Seg_copy(fillet, seg2)
		temp_v = Cross(e_line, seg2.start - seg2.center)
		fillet.type = 2 + (temp_v[1] < 0 and not seg1.type == seg2.type or temp_v[1] > 0 and seg1.type == seg2.type)
		fillet.fillet = 0
		fillet.radius = seg1.fillet
		fillet.center = c_point + i_line
		fillet.start = seg1.center + Normalize(fillet.center - seg1.center) * abs(seg1.radius)
		fillet.end = seg2.center + Normalize(fillet.center - seg2.center) * abs(seg2.radius)
		if fillet.radius < 0:
			point = fillet.start + (fillet.end - fillet.start) / 2
			fillet.center = point - (fillet.center - point)
		[fillet.angleS, fillet.angleE, fillet.angleR] = Find_angles(fillet)
		seg2.start = fillet.end
		return 1
	else:
		Seg_copy(fillet, seg2) # i.e., remove the fillet.
		fillet.number = seg2.number - 1
		g71_log("Assert, Do_fillet returned after contour segment %d without doing anything.\n" % seg1.number)
		return 0

###########################################################################
# Find_intersect
###########################################################################

#function [intersect1] = 
def	Find_intersect(seg1, seg2):
	if (seg1.type == 0 or seg1.type == 1) and (seg2.type == 0 or seg2.type == 1):
		seg1.hat = Normalize(seg1.end - seg1.start)
		cl_v = Cross(seg1.hat, seg2.start - seg1.end)
		cl_v_m = Magnitude(cl_v)
		cl_v = Cross(seg1.hat, cl_v)
		seg2_hat = Normalize(seg2.end - seg2.start)
		den = Dot(Normalize(cl_v), seg2_hat)
		if den == 0 or cl_v_m == 0:
#			intersect1 = seg2.start
			g71_log('Warning, parallel segment in Find_intersect. (semi) finishing segment %d.\n' % seg2.number)
			return seg2.start # What I want here.
		intersect1 = seg2.start + seg2_hat * cl_v_m / den
		return intersect1
	
	elif (seg1.type == 0 or seg1.type == 1) and (seg2.type == 2 or seg2.type == 3):
		seg1.hat = Normalize(seg1.end - seg1.start)
		cl_v = Cross(seg1.hat, seg2.center - seg1.end)
		cl_v_m2 = SqrMagnitude(cl_v)
		cl_v = Cross(seg1.hat, cl_v)
		i_dist_sqr = seg2.radius**2 - cl_v_m2
		if i_dist_sqr < 0:
			i_dist = 0
			if i_dist_sqr < -.0001:
				g71_log('* Error, no intersection possible after (semi) finishing segment %d.\n' % (seg2.number - 1))
				return seg2.start
		else:
			i_dist = math.sqrt(i_dist_sqr)
		intersect1 = seg2.center + cl_v + seg1.hat * i_dist
		intersect2 = seg2.center + cl_v - seg1.hat * i_dist
		if Magnitude(seg2.start - intersect2) < Magnitude(seg2.start - intersect1):
			intersect1 = intersect2
		return intersect1
	
	elif (seg1.type == 2 or seg1.type == 3) and (seg2.type == 0 or seg2.type == 1):
		seg2_hat = Normalize(seg2.end - seg2.start)
		cl_v = Cross(seg2_hat, seg1.center - seg2.end)
		cl_v_m2 = SqrMagnitude(cl_v)
		cl_v = Cross(seg2_hat, cl_v)
		i_dist_sqr = seg1.radius**2 - cl_v_m2
		if i_dist_sqr < 0:
			i_dist = 0
			if i_dist_sqr < -.0001:
				g71_log('** Error, no intersection possible after (semi) finishing segment %d.\n' % (seg2.number - 1))
				return seg2.start
		else:
			i_dist = math.sqrt(i_dist_sqr)
		intersect1 = seg1.center + cl_v + seg2_hat * i_dist
		intersect2 = seg1.center + cl_v - seg2_hat * i_dist
		if Magnitude(seg1.end - intersect2) < Magnitude(seg1.end - intersect1):
			intersect1 = intersect2
		return intersect1
	
	elif (seg1.type == 2 or seg1.type == 3) and (seg2.type == 2 or seg2.type == 3):
		c_line = seg2.center - seg1.center
		c_line_m = Magnitude(c_line)
		koll = c_line_m - (abs(seg1.radius) + abs(seg2.radius))
		if koll > 0:
			c_line_m = abs(seg1.radius) + abs(seg2.radius)
			if koll < -.0001:
				g71_log('*** Error, no intersection possible after (semi) finishing segment %d.\n' % (seg2.number - 1))
				return seg2.start
				
		c_line_hat = Normalize(c_line)
		l1 = (c_line_m**2 + seg1.radius**2 - seg2.radius**2) / 2 / c_line_m
		c_point = seg1.center + c_line_hat * l1
		i_dist_sqr = seg1.radius**2 - l1**2
		if i_dist_sqr < 0:
			i_dist = 0
			if i_dist_sqr < -.0001:
				g71_log('**** Error, no intersection possible after (semi) finishing segment %d.\n' % (seg2.number - 1))
				return seg2.start
		else:
			i_dist = math.sqrt(i_dist_sqr)
		i_line_hat = Cross(c_line_hat, [0, -1, 0])
#		i_line_hat = RotateY(c_line_hat, math.pi / 2)
		i_line = i_line_hat * i_dist
	
		e_line = seg1.end - seg1.center
		ce_cross = Cross(c_line, e_line)
		if ce_cross[1] < 0:
			i_line = -i_line
		intersect1 = c_point + i_line
		return intersect1
	else:
		g71_log("Assert, Find_intersect returned after (semi) finishing segment %d without doing anything.\n" % (seg2.number - 1))
		return seg2.start # Maybe halt here instead.

##################################################################
# Seg_copy
##################################################################
	
def Seg_copy(dest, source):
	dest.type = source.type
	dest.start = source.start
	dest.end = source.end
	dest.radius = source.radius
	dest.center = source.center
	dest.angleS = source.angleS
	dest.angleE = source.angleE
	dest.angleR = source.angleR
	dest.fillet = source.fillet
	dest.number = source.number
	dest.f_code = source.f_code
	dest.s_code = source.s_code
	dest.m_code = source.m_code
	
###########################################################################
# Find_angles
###########################################################################

def Find_angles(seg):
	if seg.type == 0 or seg.type == 1:
		angleS = math.atan2(seg.end[0] - seg.start[0], seg.end[2] - seg.start[2]) - math.pi / 2
		angleE = angleS
		angleR = 0
	elif seg.type == 2 or seg.type == 3:
		angleS = math.atan2(seg.start[0] - seg.center[0], seg.start[2] - seg.center[2])
		angleE = math.atan2(seg.end[0] - seg.center[0], seg.end[2] - seg.center[2])
		if Magnitude(seg.end - seg.start) >= 2 * abs(seg.radius):
			angleR = math.pi * (seg.type - 2.5) * 2
			g71_log("Warning, segment %d, radius too small in Find_center, seg.radius = %f, Magnitude(seg.end - seg.start) / 2 = %f.\n" % (seg.number, seg.radius, Magnitude(seg.end - seg.start) / 2))
		else:
			angleR = AngleSigned(seg.start - seg.center, seg.end - seg.center, [0, 1, 0])

		if angleR > 0 and seg.type == 2:
			angleR = -(2 * math.pi - angleR)
		elif angleR < 0 and seg.type == 3:
			angleR = 2 * math.pi + angleR
	else:
		g71_log("Assert, Find_angles returned after segment %d without doing anything.\n" % (seg.number - 1))
		return [0,0,0]
	return [angleS, angleE, angleR]

###########################################################################
# Find_center
###########################################################################

def Find_center(seg):
	if seg.type == 0 or seg.type == 1:
		center = (seg.end + seg.start) / 2
		return center
	elif seg.type == 2 or seg.type == 3:
		a = seg.end - seg.start
		a_m = Magnitude(a)
		if a_m == 0:
			center = seg.end
			g71_log("Error, segment %d, zero angle radius in Find_center.\n" % seg.number)
			return center
		elif a_m > 2 * abs(seg.radius):
			g71_log("Warning, segment %d, radius too small in Find_center, overriding radius from %f to %f.\n" % (seg.number, seg.radius, .5 * a_m))
			seg.radius = .5 * a_m

		d = math.sqrt(seg.radius**2 - (.5 * a_m)**2) #TODO, guard this.
		d_vector_hat = a / a_m
		d_vector = Cross(d_vector_hat, [0, -sign(seg.radius), 0]) * d
#		d_vector = RotateY(a/a_m, math.pi/2 * sign(seg.radius)) * d
		a_middle = (seg.end + seg.start) / 2
		center = a_middle + d_vector * (seg.type - 2.5) * 2 + 1e-9 # sys.float_info.epsilon * something
		return center
	else:
		g71_log("Assert, Segment %d, segment type error in Find_center.\n" % seg.number)
		return V3(0, 0, 0)

##################################################################
# Sign
##################################################################

def Sign(a):
	return (a >= 0) * 2 - 1
	
###########################################################################
# 
###########################################################################



