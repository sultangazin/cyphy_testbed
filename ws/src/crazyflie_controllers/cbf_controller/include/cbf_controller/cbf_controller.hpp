#include <Eigen/Dense>
#include <mutex>
#include <string>
#include <unordered_map>


struct ObstData {
	Eigen::Vector3d position;
	Eigen::Vector3d velocity;
};

class CBFController {
	public:
		CBFController();
		~CBFController();

		// Setters
		void setTranslationState(const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a);
		void setAttitudeState(const Eigen::Quaterniond& q, const Eigen::Vector3d& w);
		void setTranslationRef(const Eigen::Vector3d& p, const  Eigen::Vector3d& v, const Eigen::Vector3d& a);
		void setAttitudeRef(const Eigen::Quaterniond& q);

		// Update obstacle
		void updateObstacles(int obst_id, const Eigen::Vector3d& p, const Eigen::Vector3d& v);

		// Step
		void step(double dt);

		// Getters
		double getControlThrust();
		Eigen::Vector3d getControlRates();


		// Setters
		void setK(std::string name, double k);
		void setVehicleMass(double m);

	private:
		// Control
		double thrust_;
		Eigen::Vector3d rates_;
		Eigen::Vector3d virtual_f_;

		// Gains
		std::unordered_map<std::string, double> Kmap_;

		// Vehicle information
		double Mass_;
			
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

		// Mutex
		std::mutex mx_;

		Eigen::Quaterniond computeAttError(Eigen::Vector3d& ve);
		void generateSafetyConstraints();

		Eigen::Vector3d solvePositionQP();
		void solveAttitudeCtrl(Eigen::Quaterniond& qerr);

		// Map
		std::unordered_map<int, ObstData> obst_map_;
};
