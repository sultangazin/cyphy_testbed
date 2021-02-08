#include <Eigen/Dense>
#include <vector>
#include <mutex>

struct SimParams {
	double Mass;
	double c_drag;
	double a_drag;
};


// Interface Class
class IDynamics {
	public:
		IDynamics();
		virtual ~IDynamics() = 0;

		virtual void step(double dt) = 0;

		virtual void set_state(const std::vector<double>& x0) = 0;
		virtual void set_inputs(const std::vector<double>& u) = 0;

		virtual std::vector<double> get_state() = 0;
		virtual std::vector<double> get_acceleration() = 0;
};


class Dynamics_UAngles : public IDynamics {
	public:
		Dynamics_UAngles(const SimParams& sp);
		~Dynamics_UAngles();

		void step(double dt);

		virtual void set_state(const std::vector<double>& x0);
		void set_inputs(const std::vector<double>& u);

		std::vector<double> get_state();
		std::vector<double> get_acceleration();

	private:
		int StateDim_;
		int InputDim_;
		int MeasDim_;
		int PerfDim_;

		mutable std::mutex data_mx_;

		SimParams params_;

		// State
		Eigen::Vector3d pos_;
		Eigen::Vector3d	vel_;
		Eigen::Vector3d acc_;
		Eigen::Quaterniond quat_;

		// Controls
		double thrust_;
		Eigen::Vector3d rpy_;
};


class Dynamics_URates : public IDynamics {
	public:
		Dynamics_URates(const SimParams& sp);
		~Dynamics_URates();

		void step(double dt);

		virtual void set_state(const std::vector<double>& x0);
		void set_inputs(const std::vector<double>& u);

		std::vector<double> get_state();
		std::vector<double> get_acceleration();

	private:

		int StateDim_;
		int InputDim_;
		int MeasDim_;
		int PerfDim_;

		mutable std::mutex data_mx_;

		SimParams params_;

		// State
		Eigen::Vector3d pos_;
		Eigen::Vector3d	vel_;
		Eigen::Vector3d acc_;
		Eigen::Quaterniond quat_;

		// Controls
		double thrust_;
		Eigen::Vector3d b_angvel_;
};
