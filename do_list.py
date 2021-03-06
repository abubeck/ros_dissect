
import matplotlib.pyplot as plt
import shutil
from code_analyse import *

packages = ["cob_base_velocity_smoother","cob_collision_velocity_filter", "cob_footprint_observer", "cob_3d_fov_segmentation", "cob_read_text", "cob_marker", "cob_kuka_rsi", "cob_kinematics", "cob_linear_nav", "cob_trajectory_controller", "cob_sick_s300", "cob_phidgets", "cob_voltage_control"]

if __name__ == "__main__":
	w = RosWiki()
	f = open('results/result.csv', 'w')
	f.write( "package, lloc, roslloc, sep\n")
	package_names = []
	llocs = []
	rosllocs = []
	seps = []
	for package in packages:
		print "======================="+ package +"======================================"
		p = Parser(package)
		w.pretty_print(p.parsed_lines)
		[lloc, num_of_param, num_of_comm] = w.code_to_ros_ratio(p.parsed_lines)
		w.model_print(p.parsed_lines)
		sep = w.analyse_separation_metric(p.parsed_lines, num_of_param, num_of_comm, lloc-num_of_param-num_of_comm)
		shutil.copyfile("code_colored.html", "results/"+package+".html")
		shutil.copyfile("generated.ros_package", "results/"+package+".ros_package")
		f.write( package+", "+str(lloc)+", "+str(num_of_param + num_of_comm)+", " + str(sep) + "\n")
		package_names.append(package)
		llocs.append(lloc)
		rosllocs.append(num_of_param + num_of_comm)
		seps.append(sep)
		print "========================================================================"
		print ""
	f.close()
	if(sys.argv[1] == "fig"):
		print package_names
		print llocs
		print rosllocs
		fig = plt.figure()
		ax = fig.add_subplot(1,1,1)
		x = range(len(package_names))
		#ax.bar(x, llocs,color='#F90101', align='center')
		#ax.bar(x, rosllocs,color='#00933B', align='center')
		ax.bar(x, seps,color='#0266C8', align='center')
		ax.set_xticks(x)
		ax.set_xticks(x)
		ax.set_xticklabels(package_names)
		ax.set_title('Code amount, by package',fontstyle='italic')
		plt.ylabel('Logical lines of code')
		fig.set_facecolor('white')
		fig.autofmt_xdate()
		plt.show()