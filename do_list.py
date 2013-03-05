
import shutil
from code_analyse import *

packages = ["cob_base_velocity_smoother","cob_collision_velocity_filter", "cob_footprint_observer", "cob_3d_fov_segmentation", "cob_read_text", "cob_marker"]

if __name__ == "__main__":
	w = RosWiki()
	f = open('results/result.csv', 'w')
	f.write( "package, lloc, roslloc, sep\n")
	for package in packages:
		print "======================="+ package +"======================================"
		p = Parser(package)
		w.pretty_print(p.parsed_lines)
		[lloc, roslloc] = w.code_to_ros_ratio(p.parsed_lines)
		w.model_print(p.parsed_lines)
		shutil.copyfile("code_colored.html", "results/"+package+".html")
		shutil.copyfile("generated.ros_package", "results/"+package+".ros_package")
		f.write( package+", "+str(lloc)+", "+str(roslloc)+", sep\n")
		print "========================================================================"
		print ""
	f.close()