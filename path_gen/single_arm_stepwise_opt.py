import sys
sys.path.append('../')
from constraint_solver import *


def main():
	dataset='wood/'
	solution_dir='curve_pose_opt3/'
	curve = read_csv("../../data/"+dataset+solution_dir+"Curve_in_base_frame.csv",header=None).values
	# curve_js = read_csv("../../data/"+dataset+solution_dir+"Curve_js.csv",header=None).values
	# q_init=curve_js[0]


	q_init=[0.085918203,	0.096852813,	0.284197147,	2.563882607,	-1.344704035,	-3.032035596]

	robot=abb6640(d=50)
	opt=lambda_opt(curve[:,:3],curve[:,3:],robot1=robot,steps=len(curve))
	

	q_out=opt.single_arm_stepwise_optimize(q_init)

	####output to trajectory csv
	df=DataFrame({'q0':q_out[:,0],'q1':q_out[:,1],'q2':q_out[:,2],'q3':q_out[:,3],'q4':q_out[:,4],'q5':q_out[:,5]})
	df.to_csv('trajectory/stepwise_opt/arm1.csv',header=False,index=False)

	dlam_out=calc_lamdot(q_out,opt.lam,opt.robot1,1)


	plt.plot(opt.lam,dlam_out,label="lambda_dot_max")
	plt.xlabel("lambda")
	plt.ylabel("lambda_dot")
	plt.ylim([0,2000])
	plt.title("max lambda_dot vs lambda (path index)")
	plt.savefig("trajectory/stepwise_opt/results.png")
	plt.show()
	

if __name__ == "__main__":
	main()