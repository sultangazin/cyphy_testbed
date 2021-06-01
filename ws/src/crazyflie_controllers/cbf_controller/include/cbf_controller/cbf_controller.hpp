#include <Eigen/Dense>
#include <mutex>


class CBFController {
	public:
		CBFController();
		~CBFController();

		// Setters
		void setPosState(const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a);
		void setAttState(const Eigen::Quaterniond& q);

		void setPosRef(const Eigen::Vector3d& p, const  Eigen::Vector3d& v, const Eigen::Vector3d& a);
		void setAttRef(const Eigen::Vector3d& q);

		// Step
		void step();

		// Getters
		double getThrust();
		Eigen::Vector3d getTorques();

	private:
		// Control
		double thrust_;
		Eigen::Vector3d torques_;

		Eigen::Vector3d virtual_f_;
	
			
		// State
		Eigen::Vector3d position_;
		Eigen::Vector3d velocity_;
		Eigen::Vector3d acceleration_;
		Eigen::Quaterniond quaternion_;
		Eigen::Vector3d ang_velocity_;

		// Reference
		Eigen::Vector3d ref_position_;
		Eigen::Vector3d ref_velocity_;
		Eigen::Vector3d ref_acceleration_;
		Eigen::Quaterniond ref_quaternion_;

		// Mutex
		std::mutex mx_;

		void computeTrackingError();

		void solvePositionQP();
		void solveAttitudeQP();
};
